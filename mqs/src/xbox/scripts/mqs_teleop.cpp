#include <ros/ros.h>
#include <xbox/land.h>
#include <xbox/marine.h>
#include <xbox/op.h>
#include <sensor_msgs/Joy.h>
#include <xbee/cmd_ctrl.h>
 

 class TeleopMQS
 {
 public:
   TeleopMQS();
 
 private:
   void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
 
   ros::NodeHandle nh_;
 
   int fwd,rev,strl,thl,strm,esc,bp,daq,wrt,cp,rvm,abort,start;
   int fwd_scale, rev_scale,strl_scale,thl_scale,strm_scale;
   int fwd_shift, rev_shift,strl_shift,thl_shift,strm_shift;
   int fwd_lim;
   //initialize the swtiches off
   bool esc_on=false;
   bool bp_on=false;
   bool daq_on=false;
   bool cp_on=false;
   bool wrt_on=false;
   bool start_on=false;
   bool abort_on=false;
   int prev_[6]={0,0,0,0,0,0};
   bool init_=true; 
   ros::Publisher cmd_ctrl_pub;
   ros::Publisher marine_joy_pub;
   ros::Publisher land_joy_pub;
   ros::Publisher op_joy_pub;
   ros::Subscriber joy_sub_;
 
 };
 

 TeleopMQS::TeleopMQS():
  strm(0), //marine steering on LS <->
  rev(2), //land rev on LT , was (5)
  strl(3), //land steer on RS <->, was (2)  
  thl(4), //marine throttle on RS ^ only, was (3)
  fwd(5), //land fwd on RT, was (4)
  esc(0), //esc binary on A
  bp(1), //bilge pump binary on B
  daq(2), //daq binary on X
  cp(3), //cooling pump binary on Y
  wrt(4), //wheel retraction binary on LB
  rvm(5), //marine reverse on RB
  abort(8), //abort to RC transmitter on Xbox button
  start(7) //start queued manuever
 {
   //setting up parameters for joystick, sets defualt to
   //the initial value for each param to centered (127) or zero 
   nh_.param("steer_marine",strm,strm);
   nh_.param("reverse", rev,rev);
   nh_.param("steer_land", strl,strl);
   nh_.param("throttle", thl,thl);
   nh_.param("forward",fwd,fwd);
  //scaling joystick inputs
   nh_.param("scale_steer_marine",strm_scale,strm_scale);
   nh_.param("scale_reverse", rev_scale,rev_scale);
   nh_.param("scale_steer_land", strl_scale,strl_scale);
   nh_.param("scale_throttle", thl_scale,thl_scale);
   nh_.param("scale_forward",fwd_scale,fwd_scale);
   nh_.param("forward_limit", fwd_lim,fwd_lim);
  //shifting joystick inputs
   nh_.param("shift_steer_marine",strm_shift,strm_shift);
   nh_.param("shift_reverse",rev_shift,rev_shift);
   nh_.param("shift_steer_land",strl_shift,strl_shift);
   nh_.param("shift_throttle",thl_shift,thl_shift);
   nh_.param("shift_forward",fwd_shift,fwd_shift);


   //publish message to topic cmd_ctrl, queue size is 2 messages
   cmd_ctrl_pub = nh_.advertise<xbee::cmd_ctrl>("cmd_ctrls", 2);

   //publish each suboperation for force-feedback node
   marine_joy_pub = nh_.advertise<xbox::marine>("marine",10);
   land_joy_pub = nh_.advertise<xbox::land>("land",10);
   op_joy_pub = nh_.advertise<xbox::op>("op",10);

 
 
   joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopMQS::joyCallback, this);

 }
 
 void TeleopMQS::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
 {
   //writes axes and buttons from joystick to the messages for each suboperation
   xbox::land land_joy; 
   xbox::marine marine_joy;
   xbox::op op_joy;
   marine_joy.strm = joy->axes[strm];
   marine_joy.thl = joy->axes[thl];
   land_joy.fwd = joy->axes[fwd];
   land_joy.rev = joy->axes[rev];
   land_joy.strl = joy->axes[strl];
   op_joy.esc=joy->buttons[esc]; //switch
   op_joy.bp=joy->buttons[bp]; //switch
   op_joy.daq=joy->buttons[daq]; //switch
   op_joy.wrt=joy->buttons[wrt]; //momentary
   op_joy.cp=joy->buttons[cp]; //switch
   op_joy.rvm=joy->buttons[rvm]; //momentary
   op_joy.start=joy->axes[start]; //switch for maneuver
   op_joy.abort=joy->buttons[abort]; //switch, permanent abort for pc operation

  //initializes previous value first time 
  if(init_==true)
  {
    prev_[0]=op_joy.esc;
    prev_[1]=op_joy.bp;
    prev_[2]=op_joy.daq;
    prev_[3]=op_joy.cp;
    prev_[4]=op_joy.wrt;
    prev_[5]=op_joy.start;
    init_= false;
  }

   //place each published message into an array container for cmd_ctrl
   xbee::cmd_ctrl cmd_ctrl_;
   if(nh_.getParam("scale_throttle", thl_scale))
   {
     ROS_INFO_STREAM("Max throttle set to: " << thl_scale);
   }
   // set forward drive speed limit
   if(nh_.getParam("forward_limit", fwd_lim))
   {
    fwd_scale = (127 - fwd_lim)/2;
    fwd_shift = 127 - fwd_scale;
    nh_.setParam("fwd_scale", fwd_scale); // we reset them on the server as well
    nh_.setParam("fwd_shift",fwd_shift);
   }


   {
    //int() converts float values from joy.msg to byte values
    //strm on channel 0
    cmd_ctrl_.cmd_ctrls[0]=int(strm_scale*marine_joy.strm+strm_shift);
    
    //fwd/rev on channel 1 mqs_ctrl_.cmds[i] = 0;
    if(land_joy.fwd<1) //only input on LT
    {
      //go forward
      cmd_ctrl_.cmd_ctrls[1]=int(fwd_scale*land_joy.fwd+fwd_shift);
    }
    else if (land_joy.rev<1) //only input on RT
    {
      //if the reverse trigger only is pressed
      cmd_ctrl_.cmd_ctrls[1]=int(rev_scale*land_joy.rev+rev_shift);
    }
    else if (land_joy.fwd<1 && land_joy.rev<1)
    {
      //if they're both pressed do nothing, sorry no burnouts :(
      cmd_ctrl_.cmd_ctrls[1]=127;
    }
    if (land_joy.fwd>=1 && land_joy.rev>=1) //if neither are pressed do nothing, actually not sure how necessary this one is
    {
      cmd_ctrl_.cmd_ctrls[1]=127;
    }
    //marine throttle on channel 2
    if(marine_joy.thl<=0) //if the throttle reads negative set to zero
    {
      cmd_ctrl_.cmd_ctrls[2]=int(thl_scale*0+thl_shift);
    }
    else
    {
      cmd_ctrl_.cmd_ctrls[2]=int(thl_scale*marine_joy.thl+thl_shift);
    }

    //strl on channel 3
    cmd_ctrl_.cmd_ctrls[3]=int(strl_scale*land_joy.strl+strl_shift);
    
    //on-off switch for esc on channel 4
    if (op_joy.esc==1 && prev_[0]==0) //if esc button is pressed the first time turn it on
    {
      esc_on= !esc_on;
      cmd_ctrl_.cmd_ctrls[4]=op_joy.esc;
    }
    if (esc_on==true)
    {
      cmd_ctrl_.cmd_ctrls[4]=1; //hold esc on
    }
    else
    {
      cmd_ctrl_.cmd_ctrls[4]=0;
    }

    //on-off switch for bilge pump on channel 5
    if (op_joy.bp==1 && prev_[1]==0) //if bp button is pressed the first time turn it on
    {
      bp_on= !bp_on;
      cmd_ctrl_.cmd_ctrls[5]=op_joy.bp;
    }
    if (bp_on==true)
    {
      cmd_ctrl_.cmd_ctrls[5]=1; //hold bp on
    }
    else
    {
      cmd_ctrl_.cmd_ctrls[5]=0;
    }

    //on-off switch for DAQ on channel 6
    if (op_joy.daq==1 && prev_[2]==0) //if daq button is pressed the first time turn it on
    {
      daq_on= !daq_on;
      cmd_ctrl_.cmd_ctrls[6]=op_joy.daq;
    }
    if (daq_on==true)
    {
      cmd_ctrl_.cmd_ctrls[6]=1; //hold daq on
    }
    else
    {
      cmd_ctrl_.cmd_ctrls[6]=0;
    }
    //on-off switch for wheel retraction on channel 7
    if (op_joy.wrt==1 && prev_[4]==0) //if wrt button is pressed the first time turn it on
    {
      wrt_on= !wrt_on;
      cmd_ctrl_.cmd_ctrls[7]=op_joy.wrt;
    }
    if (wrt_on==true)
    {
      cmd_ctrl_.cmd_ctrls[7]=1; //hold wrt on
    }
    else
    {
      cmd_ctrl_.cmd_ctrls[7]=0;
    }

    //on-off switch for cooling pump on channel 8
    if (op_joy.cp==1 && prev_[3]==0) //if cp button is pressed the first time turn it on
    {
      cp_on= !cp_on;
      cmd_ctrl_.cmd_ctrls[8]=op_joy.cp;
    }
    if (cp_on==true)
    {
      cmd_ctrl_.cmd_ctrls[8]=1; //hold cp on
    }
    else
    {
      cmd_ctrl_.cmd_ctrls[8]=0;
    }

    //rvm on channel 9
    cmd_ctrl_.cmd_ctrls[9]=op_joy.rvm;

    //abort to RC transmitter on channel 10
    if(op_joy.abort == 1)
    {
      abort_on = true;
      cmd_ctrl_.cmd_ctrls[10]=op_joy.abort; //If this is true joystick operation will be switched off to the arduino
      ROS_WARN("Joystick Interupted by ABORT\n");
    }
    if(abort_on)
    {
      cmd_ctrl_.cmd_ctrls[10]= 1; // hold on
      
      ROS_WARN("Control on R/C\n");
    }
    else
    {
      cmd_ctrl_.cmd_ctrls[10]= 0; // keep off
    }
    
    
    //on-off switch for start manuever
    if (op_joy.start==1 && prev_[5]==0) //if start button is pressed the first time turn it on
    {
      start_on= !start_on;
      cmd_ctrl_.cmd_ctrls[11]=op_joy.start;
    }
    if (start_on==true)
    {
      cmd_ctrl_.cmd_ctrls[11]=1; //hold on
    }
    else
    {
      cmd_ctrl_.cmd_ctrls[11]=0;
    }
    //last 4 channels are open
    cmd_ctrl_.cmd_ctrls[12]=0;
    cmd_ctrl_.cmd_ctrls[13]=0;
    cmd_ctrl_.cmd_ctrls[14]=0;
    cmd_ctrl_.cmd_ctrls[15]=0;

   }
   // publish the built message
   cmd_ctrl_pub.publish(cmd_ctrl_);

   
   prev_[0]=op_joy.esc;
   prev_[1]=op_joy.bp;
   prev_[2]=op_joy.daq;
   prev_[3]=op_joy.cp;
   prev_[4]=op_joy.wrt;
   prev_[5]=op_joy.start;

   // publish the suboperation messages
   marine_joy_pub.publish(marine_joy);
   land_joy_pub.publish(land_joy);
   op_joy_pub.publish(op_joy);

  }
 
 int main(int argc, char** argv)
 {

   ros::init(argc, argv, "mqs_teleop");
   TeleopMQS mqs_teleop;
   
   ros::spin();
}
