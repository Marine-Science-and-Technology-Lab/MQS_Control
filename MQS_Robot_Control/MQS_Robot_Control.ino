/*
  Model Quadski Robot control code
  Developed by Mike Swafford
  michael-swafford@uiowa.edu

  Copyright (c) 2020 IIHR-Hydroscience & Engineering
  **************************************************
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
  and associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute,
  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or
  substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
  DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  **************************************************
  
  This code is able to control the model quadski robot either directly from an SBUS RC transmitter or with a 802.15.4 protocol XBee3 communicating with a
  ROS program that can be driven manually via an xbox controller, a keyboard, or semi-autonomously with a SIMULINK feed-forward control model.
  
  The program uses a switch case to switch between either the RC transmitter or the XBee/ROS operation, control can always be pulled from the XBee/ROS by flipping 
  the approriate switch on the RC transmitter. The main code deals with reading the approriate communication packet and then divying up Servo operation that are 
  managed through individual functions for land turning, marine turning, throttle, etc... 
  
*/
#include <SBUS.h>
#include <Servo.h>
#include "Xbee.h"
#include "queue.h"

XBee xbee; //xbee RX on 0 TX on 1 for hardware UART
Queue RxQ;
/************** strm ****** fwd/rev ****** thl ****** strl ***** esc_on *** bp_on *** daq_on *** wrt_on ** cp_on ** rvm **** abort ****** 5 blank channels            ****/
float scaleC[]={3.921568627,3.921568627,3.921568627,3.921568627,1000.0000,1000.0000,1000.0000,1000.0000,1000.0000,830.0000,1000.000,3.921568627,3.921568627,3.921568627};
float shiftC[]={1000,1000,1000,1000,1000,1000,1000,1000,1000,1170,1000,1000,1000,1000,1000,1000};
//note that the scales and shifts for the Xbee are different than for the SBUS since the XBee is
//receiveing a standard bite value between 0 and 255

// a SBUS packet on serial5 RX pin 20
SBUS x8r(Serial5);
float scales[]={0.782473,0.782473,0.782473,0.782473,1.564945,1.564945,1.564945,1.564945,1.564945,1.564945,0.782473,0.782473,0.782473,0.782473,0.782473,0.782473};
float shifts[]={698.748,698.748,698.748,698.748,-602.504,-602.504,-602.504,-602.504,-602.504,-602.504,698.748,698.748,698.748,698.748,698.748,698.748};

float micsecs[16];

Servo myservos[11];

//*****************0**1**2**3**4**5**6***7***8***9***10
int Servopins[] = {2, 3, 4, 5, 6, 7, 8, 13, 15, 18, 19};
uint16_t channels[16];
bool failSafe=false;
bool lostFrame=false;
//sets PC_OP switch case to always start with comminucation over the XBee/ROS, control can be switched by either aborting on the Xbee/ROS, or pulling control on the RC transmitter
int PC_OP=1; //1=Xbee/ROS control 2=RC transmitter control
int abort_=0;

void setup() 
{
    // begin the SBUS communication
  x8r.begin(); //starts SBUS protocol on hardware serial5 at 100kbaud (RX1)
  Serial1.begin(115200); //begin coms with Xbee
  Serial.begin(9600); // Start serial comms with PC
  delay(500); //let the coms get setup before attaching servos
    for (int i = 0; i <= 10; ++i) 
    {
      
      if (i==1) //zero the land drive motors
      {
        myservos[i].attach(Servopins[i]); //attach servos at specified pins
        myservos[i].writeMicroseconds(1500);
        delay(5);
      }
      else if(i==6) //confirm wheels start in deployed position
      {
        myservos[i].attach(Servopins[i]); //attach servos at specified pins
        myservos[i].writeMicroseconds(1500); //Front retraction servos
        delay(5);

      }
      else if(i==7) //center the steering wheels
      {
        myservos[i].attach(Servopins[i]); //attach servos at specified pins
        myservos[i].writeMicroseconds(1500);
        delay(5);
      }
      else if(i==8)
      {
        myservos[i].attach(Servopins[i]); //attach servos at specified pins
        myservos[i].writeMicroseconds(1500); //rear retraction servos
        delay(5);
      }
      else
      {
        myservos[i].attach(Servopins[i]); //attach servos at specified pins
        myservos[i].writeMicroseconds(1000); //center all remaining servos
        delay(5);
      }
      //Serial.println(myservos[i].attached());
    }
  pinMode(11,OUTPUT); //set pin 11 to HIGH/LOW out put for data logger recording of wheel state. HIGH==retracted, LOW==Deployed
  pinMode(10,OUTPUT); //set pin 10 to PWM analog output for data logger reocording of land steering
  delay(5000); // give everything a moment to boot up

}

void loop() 
{
    switch(PC_OP)
  {
    case 1:
      {
        //this if statement checks the abort channel on the RC transmitter and is able to pull control to case 2 if the switch is flipped
        if (x8r.read(&channels[0], &failSafe, &lostFrame)) //if a good message on the sbus exists
        {
          //read value on channel 10 and assign it to the abort variable 
          abort_=channels[10];
          if(abort_==1663) //if abort switch it triggered
          {
            Serial.println("Abort triggered on RC transmitter!");
            PC_OP=2; //change operation to receiver
            break;
          }
        }
        int queueLen=0;
        while (Serial1.available()>0) //while there is stuff on the Xbee
        {
          unsigned char in = (unsigned char)Serial1.read(); //read the serial data
          if(!RxQ.Enqueue(in)) //if no data is available on the xbee or the signal is lost and an abort has not been triggered
          {
            Serial.println("No message from XBee or the message is empty");
            myservos[1].writeMicroseconds(1500); //stop land drive
            myservos[2].writeMicroseconds(1000); //stop marine drive
            break;
          }
          queueLen=RxQ.Size(); //returns size of elementsn in queue
          for(int i=0;i<queueLen;i++) //for the # of elements in the queue
          {
            if(RxQ.Peek(i) == 0x7E)//look for starting deliminator
            {
              unsigned char checkBuff[Q_SIZE];//
              unsigned char msgBuff[Q_SIZE]; //defines size of msgBuff 
              int checkLen=0;
              int msgLen=0; //initialize message length to zero
              checkLen=RxQ.Copy(checkBuff, i);
              msgLen=xbee.Receive(checkBuff,checkLen,msgBuff);
              if(msgLen>0)
              {
                for(int i=0;i<16;i++) //16 channels to read
                {
                  channels[i] = msgBuff[i+8];
                  //read the first 4 channels that control servos [0,1,2,3] 
                  if(i+8<=12)
                  {
                    //scale and shift the vaule on channels to be between 1000 and 2000 PWM standard
                    micsecs[i]=(channels[i]*scaleC[i]+shiftC[i]);
                  }
                  //read channels between 4 & 10 [4,5,6,7,8,9,10]
                  if(i+8>12 && i+8<=19)
                  {
                    //scale and shift the vaule on channels to be between 1000 and 2000 PWM standard
                    micsecs[i]=(channels[i]*scaleC[i]+shiftC[i]); 
                    if(micsecs[10]==2000)//2000=1 from linux for channel 10 that's the abort channel
                    {
                      PC_OP=2; //change operation to reciever
                      break;
                    }
                  }
                  //printing values on all channels
                  Serial.print(i);
                  Serial.print(",");
                  Serial.print(micsecs[i]);
                  Serial.print(" ");
                }
                Serial.println(" ");

                //Control functions go here!
                strM(); //steering marine
                landDrive(); //land driving
                marineDrive(); //marine waterjet
                strL(); //land steering
                esc(); //esc on-off
                bp(); //bilge pump on-off
                daq(); //daq on-off
                WRT(micsecs[7]); //wheel retraction
                cp();// cooling pumps on-off
                revM(); //marine reverse
                
                /*IF feedback control is required this where it would be added
                sending servo positions back to python
                memcpy(outMsg, "SERVO POS: ",11);
                memcpy(&outMsg[11], &msgBuff[8],msgLen-9);
                frameLen=xbee.Send(outMsg,msgLen+2,outFrame,addr);
                Serial5.write(outFrame,frameLen);
                */
              }
              RxQ.Dequeue(); //making sure the queue only has the most recent message in it.
            }
          }
        }
      }
      break;
    case 2:
      {
        //look for a good SBUS packet from the reciever
        if (x8r.read(&channels[0], &failSafe, &lostFrame)) 
        {
          for (int i = 0; i <= 15; ++i)
          {  
            micsecs[i]=(channels[i]*scales[i]+shifts[i]);
            //myservos[i].writeMicroseconds(micsecs[i]); 
            //Serial.print(channels[i]);
            //Serial.print(",");
            Serial.print(i);
            Serial.print(",");  
            Serial.print(micsecs[i]);
            Serial.print(" ");
          }
          Serial.println(" ");

          //Control functions go here!
          strM(); //steering marine
          landDrive(); //land driving
          marineDrive(); //marine waterjet
          strL(); //land steering
          esc(); //esc on-off
          bp(); //bilge pump on-off
          daq(); //daq on-off 
          WRT(micsecs[7]); //wheel retraction
          cp();// cooling pumps on-off
          revM(); //marine reverse
       }
       else if(failSafe==true || lostFrame==true) //if a failsafe or lost frame is triggered
       {
          Serial.println("failsafe or LOS triggered");
          //myservos[1].writeMicroseconds(1500); //stop land drive
          //myservos[2].writeMicroseconds(1000); //stop marine drive
       }
       delay(5);
      }
      break;
    default:
      {
        Serial.println("Looks like you haven't specified a control method...");
        delay(1000); //wait a sec for the operator to put a valid value at PC_OP
      }
      break;
  }
}

void strM()
/*
 strm (steering marine)
 This function controls the steering for the waterjet nozzle on the MQS robot.
 Physical pin is on 2 for the arduino, myservos[0], micsecs [0]
 Range is limited to 1000 micsecs for right turn and 2000 micsec for left turn
*/
{
  float maxRight=1000;
  float maxLeft=2000;

  //if the value is greater than the maximum allowed left turn set to max allowed left turn
  if(micsecs[0]<= maxRight)
  {
    micsecs[0]=maxRight;
  }
  else if(micsecs[0]>= maxLeft)
  {
    micsecs[0]=maxLeft;
  }
  else
  {
    myservos[0].writeMicroseconds(micsecs[0]);  
  }
}

void landDrive()
/*
  landDrive (land driving)
  This function controls the land driving motors on the MQS robot.
  Physical pin is on 3 for the arduino, myservos[1]
  Range is limited between 1000 micsecs for reverse and 2000 micsecs for forward
  Speed limits are an option that can be enabled by changing the range function or
  programming for the ESCs

  Micsecs to wheel RPM (no load):
  1000  
  1200
  1400
  1500    0.00
  1550   49.99
  1600  252.90
  1650  417.76
  1700  453.05
  1750  509.44
  1800  529.54
  1850  541.33
  1900  548.04
  1950  562.61
  2000  572.75
*/
{
  float maxREV=1450; //limits maximum reversing speed
  float maxFWD=2000;
  float crawl=1550; //crawl forward speed
  float deadzone=1500;

  //if the value is greater than the maximum allowed reverse set to maxREV
  if(micsecs[1] <= maxREV)
  {
    micsecs[1] = maxREV;
  }
  else if(micsecs[1] > 1505 && micsecs[1] <= 1550) //if some throttle forward is asked use crawl speed
  {
    micsecs[1] = crawl;
  }
  else if(micsecs[1] >= maxFWD) //full gas if at or for some reason above full gas
  {
    micsecs[1]=maxFWD;
  }
  else if(micsecs[1]>1450 && micsecs[1]<=1505) //sets the range for the deadzone
  {
    micsecs[1]=deadzone;
  }
  else if(micsecs[7]>=1900 ) //if the wheels are up!
  {
    micsecs[1]=deadzone; //land drive is disabled
  }
  else
  {
    micsecs[1]=micsecs[1];
  }
  //write the micsecs to the motors
  myservos[1].writeMicroseconds(micsecs[1]);  
}

void marineDrive()
/*
  marineDrive (marine driving)
  This function controls the waterjet motor on the MQS robot.
  Physical pin is on 4 for the arduino, myservos[2]
  Range is limited between 1000 micsecs for not throttle and 2000 micsecs for forward
  Speed limits are an option that can be enabled by changing the range function or
  programming for the ESCs
  add write 1000 for this channel incase of LOS

  check SBUS library for failsafes
  
  Micsecs to waterjet RPM:
  1000  0.00 rpm
  1200
  1400
  1500  
  1600
  1800
  2000
*/
{
  float maxFWD=2000;

  //if the value is less than no throttle write no throttle
  if(micsecs[2]<= 1000)
  {
    micsecs[2]=1000;
  }
  else if(micsecs[2]>= maxFWD)
  {
    micsecs[2]=maxFWD;
  }
  myservos[2].writeMicroseconds(micsecs[2]);  
}

void strL()
/*
 strL (steering land)
 This function controls the steering for the front wheels on the MQS robot.
 Physical pins are 13 and 14 for the arduino, micsecs[3] & micsecs[4]
 myservos[7] for both
 For a left turn a signal of 2000 is recieved and 2000 is written to both servos
 For a right turn a signal of 1000 is recieved and 1000 written to both servos
*/
{
  float maxLeft=1200;
  float maxRight=1800;

  //if the value is greater than the maximum allowed right turn set to max allowed right turn
  if(micsecs[3]>= maxRight)
  {
    micsecs[3]=maxRight;
  }
  if(micsecs[3]<= maxLeft)
  {
    micsecs[3]=maxLeft;
  }
  myservos[7].writeMicroseconds(micsecs[3]);   
  int datalog_land_steer=(micsecs[3]/2.35294)-510; //scale micsecs down to a byte value to pass to the data logger
  analogWrite(10,datalog_land_steer);
}

void esc()
/*
  ESC on-off
  This function controls whether the ESCs on board the MQS is turned on or off. Function is
  more or less a safety feature to disable driving (land or marine) if the vehicle is powered on
  1000 value turns the esc off, 2000 value turns the esc on
  physical pin is 19, micsecs[4], myservos[10] 
 */
{
  myservos[10].writeMicroseconds(micsecs[4]);
}

void bp()
/*
  bilge pump on-off
  This function controls whether the bilge pump on board the MQS is turned on or off. This serves
  purpose of expelling any water that leaks into the MQS out.
  This may later be automated using moisture sensors
  1000 value turns the bilge pump off, 2000 value turns the bilge pump on
  physical pin is 5, micsecs[5], myservos[3] 
 */
{
  myservos[3].writeMicroseconds(micsecs[5]);
}

void daq()
/*
  Data aquisition system on-off
  This function controls whether the DAQ on board the MQS is turned on or off.
  1000 value turns the daq off, 2000 value turns the daq on
  physical pin is 18, micsecs[6], myservos[9] 
 */
{
  myservos[9].writeMicroseconds(micsecs[6]);
}

void WRT(float)
/*
 WRT (Wheel retraction)
 This function controls whether the wheels on the MQS robot are deployed or retracted. 
 Servos are split port and starboard
 Port side servos operate on pin 15, micsecs[7], myservos[8]. 
 Port side deployed 1150, port side retracted 2090
 Starboard side servos operate on pin 8, micsecs[7], myservos[6].
 Starboard side deployed 2050, starboard side retracted 1100
 
 For the signal micsecs[7]=1000 is deployed and 2000 is retracted 
*/
{
  float FrontDeployed=1500;
  float FrontRetract=2100;

  float RearDeployed=1500;
  float RearRetract=2100;
  
  //add while loop to disable WRT if in transition
    if(micsecs[7]<=1500) //wheels to deployed position
    {
      myservos[8].writeMicroseconds(FrontDeployed);
      myservos[6].writeMicroseconds(RearDeployed);
      digitalWrite(11,LOW); //write low signal to pin 11 so data logger can read that the wheels are deployed
    }
    else if(micsecs[7]>=1900) //wheels to retracted position
    {
      myservos[8].writeMicroseconds(FrontRetract);
      myservos[6].writeMicroseconds(RearRetract);
      digitalWrite(11,HIGH); //write high signal to pin 11 so data logger can read that the wheels are retracted
    }
}

void cp()
/*
  Cooling pumps on-off
  This function controls whether the cooling pumps on board the MQS is turned on or off. This serves
  purpose of cooling the water cooled components.
  CANNOT OPERATE UNTIL IN THE WATER
  1000 value turns the cooling pumps off, 2000 value turns the cooling pumps on
  physical pin is 6, micsecs[8], myservos[4] 
*/
{
  myservos[4].writeMicroseconds(micsecs[8]);
}

void revM()
/*
  revM (Reverse bucket for waterjet)
  This function controls whether the reverse bucket is up or down.
  1000 value is a raised bucket, 2000 value enables reverse
  physical pin is 7, micsecs[9], myservos[5] 
 */
{
  myservos[5].writeMicroseconds(micsecs[9]);
}
