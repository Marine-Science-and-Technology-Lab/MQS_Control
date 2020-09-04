#include <ros/ros.h>
#include <xbee/cmd_ctrl.h>
#include <xbee/auto_ctrl.h>
#include <xbee/mqs_ctrl.h>

    class HandshakeMQS
    {
        public:
            HandshakeMQS();
            void processMessages();
        private:
            void joyCallback(const xbee::cmd_ctrl::ConstPtr& cmd_ctrls);
            void autoCallback(const xbee::auto_ctrl::ConstPtr& auto_ctrls);
            
            void joystickOverride();

            ros::NodeHandle nh_;
            ros::Publisher mqs_ctrl_pub;
            ros::Subscriber cmd_ctrl_sub;
            ros::Subscriber auto_ctrl_sub;
            xbee::mqs_ctrl mqs_ctrl_; 
            int current_joy[11]={0,0,0,0,0,0,0,0,0,0,0}; //current joy positons

            //initialize all joystick channels to 0 or center
            int prev_joy[11]={0,0,0,0,0,0,0,0,0,0,0}; //array to track the previous joy positons
            bool init_=true; //init variable to set prev_joy
            int handshakemqs;
            bool fresh_joy=false; 
            int fresh_joy_count=0; 
            bool fresh_auto=false; 
            int fresh_auto_count=0; 
    };

    HandshakeMQS::HandshakeMQS():
    handshakemqs(1) //need this to get it to compile?
    {
        //publish MQS_CTRL to xbee
        mqs_ctrl_pub=nh_.advertise<xbee::mqs_ctrl>("cmds",2);

        //subscribe to auto_ctrl and cmd_ctrl
        cmd_ctrl_sub=nh_.subscribe<xbee::cmd_ctrl>("cmd_ctrls",2, &HandshakeMQS::joyCallback,this);
        auto_ctrl_sub=nh_.subscribe<xbee::auto_ctrl>("auto_ctrls",2, &HandshakeMQS::autoCallback,this);

        //   set up initial values for publishing because callbacks are not called
        //   until ros::spin... 
        mqs_ctrl_.cmds[0]=127; //set wheels to initialize centered
        mqs_ctrl_.cmds[1]=127; //set land motors to initialize as stopped
        mqs_ctrl_.cmds[2]=0;  //set marine motor to initialize as stopped
        mqs_ctrl_.cmds[3]=127; //set land steering to initilize centered
        mqs_ctrl_.cmds[4]=0; //set esc to off
        mqs_ctrl_.cmds[5]=0; //set bilge pump to off
        mqs_ctrl_.cmds[6]=0; //set DAQ to off
        mqs_ctrl_.cmds[7]=0; //set wheel retraction to deployed
        mqs_ctrl_.cmds[8]=0; //set cooling pump to off
        mqs_ctrl_.cmds[9]=0; //set marine rev to disabled
        mqs_ctrl_.cmds[10]=0; //set abort to off
    }

    void HandshakeMQS::processMessages()
    {
        if(fresh_auto==true)
        {
            joystickOverride();

            //set fresh to false
            fresh_auto=false;
            fresh_auto_count = 0;
        }
        else 
        {
            fresh_auto_count += 1;
            if(fresh_auto_count > 3)
            {
                ROS_INFO("Auto Strike out occured...preparring to abort!");
                //trigger an abort if a strike out occurs
                mqs_ctrl_.cmds[10]=1;
                joystickOverride();
            }
            else 
            {
                // Auto message was not received -- we still need to process Joystick
                ROS_INFO("Auto Message stale, freshness count: [%d]", fresh_auto_count);
                joystickOverride();
            }
        }
        // Publish 
        mqs_ctrl_pub.publish(mqs_ctrl_);
    }

    void HandshakeMQS::joystickOverride() 
    {

        if(fresh_joy==true)
        {
            //Decide whether the value has changed and should be passed on...
            for(int i=0; i<=10;i++)
            {
                //if a command has changed on the joystick
                if(current_joy[i] != prev_joy[i])
                {
                    mqs_ctrl_.cmds[i]=current_joy[i]; //overwrites master if something on joy changes
                    prev_joy[i]=current_joy[i]; //reset the specific joy command to the new value    
                }       
            }
            fresh_joy = false;
            fresh_joy_count = 0;
        }
        else
        {
            fresh_joy_count += 1;
            if(fresh_joy_count > 3)
            {
                ROS_INFO("Joystick Strike out occured...preparring to abort!");
                //trigger an abort if a strike out occurs
                mqs_ctrl_.cmds[10]=1;

            }
            else 
            {
                // Joystick message was not received -- we still need to process Joystick
                ROS_INFO("Joystick Message stale, freshness count: [%d]", fresh_joy_count);
            }
        }
    }

    void HandshakeMQS::joyCallback(const xbee::cmd_ctrl::ConstPtr& cmd_ctrls)
    {
        //initialize the initial position for all joystick or keyboard messages
        //handshake will only let through changes from these or the previous joy change
        if(init_==true)
        {
            prev_joy[0]=127; //set wheels to initialize centered
            prev_joy[1]=127; //set land motors to initialize as stopped
            prev_joy[2]=0;  //set marine motor to initialize as stopped
            prev_joy[3]=127; //set land steering to initilize centered
            prev_joy[4]=0; //set esc to off
            prev_joy[5]=0; //set bilge pump to off
            prev_joy[6]=0; //set DAQ to off
            prev_joy[7]=0; //set wheel retraction to deployed
            prev_joy[8]=0; //set cooling pump to off
            prev_joy[9]=0; //set marine rev to disabled
            prev_joy[10]=0; //set abort to off
            init_= false;
        }

        for(int i=0; i<=10;i++)
        {
            current_joy[i] = cmd_ctrls->cmd_ctrls[i];
        }
        fresh_joy = true;
    }

    void HandshakeMQS::autoCallback(const xbee::auto_ctrl::ConstPtr& auto_ctrls)
    {
        fresh_auto=true;

        for(int i=0;i<=10;i++)
        {
            //write all of the auto command to the master message
            mqs_ctrl_.cmds[i]=auto_ctrls->auto_ctrls[i];
        }
    }

 int main(int argc, char** argv)
 {

   ros::init(argc, argv, "mqs_handshake");
   HandshakeMQS mqs_handshake;

   ros::Rate loop_rate(100); // 100Hz to match xbee data rate

   while(ros::ok())
   {
        mqs_handshake.processMessages();

        ros::spinOnce();
        loop_rate.sleep();
   }
 }
