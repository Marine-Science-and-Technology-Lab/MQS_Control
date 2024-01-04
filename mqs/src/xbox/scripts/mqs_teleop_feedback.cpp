#include <ros/ros.h>
#include <xbox/land.h>
#include <xbox/marine.h>
#include <xbox/op.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <string>
#include <unordered_map>
#include <std_msgs/Float32.h>


class JoystickFeedback 
{
    public:
        JoystickFeedback(): init_(true) 
        {
            // Init ROS node handle
            nh_ = ros::NodeHandle();

            // Subscribe to each joystic input
            marine_sub = nh_.subscribe<xbox::marine>("marine",10, &JoystickFeedback::marineCallback, this);
            land_sub = nh_.subscribe<xbox::land>("land",10, &JoystickFeedback::landCallback, this);
            op_sub = nh_.subscribe<xbox::op>("op",10, &JoystickFeedback::opCallback,this);

            // Subscribe to the Mission Ellapsed Time
            met_sub=nh_.subscribe<std_msgs::Float32>("MET",2, &JoystickFeedback::metCallback,this);

            // Publish the haptic feedback
            haptic_feedback_pub = nh_.advertise<sensor_msgs::JoyFeedbackArray>("joy/set_feedback",10);

            // parameter for user to scale the 100% force feedback by
            nh_.param("force_feedback",scale_feedback_,100.0);
            scale_feedback_ = scale_feedback_/100.0; // convert from % to decimal
            

        }

        
    private:
        ros::NodeHandle nh_;
        ros::Subscriber marine_sub;
        ros::Subscriber land_sub;
        ros::Subscriber op_sub;
        ros::Subscriber met_sub;
        ros::Publisher haptic_feedback_pub;

        bool init_; 
        int prev_[5]={0,0,0,0,0};
        double scale_feedback_;
        float MET_END;
        float mqs_met;

        static const std::unordered_map<char,std::string> morseCodeMap;
        


        void marineCallback(const xbox::marine::ConstPtr& marine)
        {
            // if throttle moved vibrate controller on id 1 with intensity == value
            if (marine->thl > 0.0)
            {
                waterjet_feedback(marine->thl);
            }
            else
            {
                // we need to tell waterjet_feedback to turn off
                waterjet_feedback(0.0);
            }
        }

        void landCallback(const xbox::land::ConstPtr& land)
        {
            // Map from [-1,1] to [0,1]
            // When trigger is fully depressed, (1-land->fwd)/2 will be 1
            // When there is no input, (1-land->fwd)/2 will be 0
            if ((1-land->fwd)/2 > 0.1)
            {
                land_fwd_feedback((1-land->fwd)/2-0.1); // substract the offset
            }
            else if ((1-land->rev) > 0.05)
            {
                land_rev_feedback((1-land->rev)/2);
            }
            else
            {
                land_fwd_feedback(0.0);
            }
        }

        void opCallback(const xbox::op::ConstPtr& op)
        {
            if (init_)
            {
                //initialize the swtiches off
                prev_[0]=op->bp;
                prev_[1]=op->daq;
                prev_[2]=op->cp;
                prev_[3]=op->wrt;
                prev_[4]=op->start;
                init_= false;
            }
            // bilge pump
            if (op->bp==1 && prev_[0]==0) //if button is pressed the first time turn it on
            {
                //bp turned on do button buzz
                sendPulse(500);
                prev_[0] = 1;
            }
            else if (op->bp==1 && prev_[0] == 1)
            {
                prev_[0] = 0;
            }
            if (op->daq == 1 && prev_[1]==0)
            {
                //probably won't do one for this?
            }
            if (op->cp==1 && prev_[2] == 0)
            {
                //cp turned on do button buzz
                sendPulse(500);
                prev_[2] = 1;
            }
            else if (op->cp==1 && prev_[2]==1)
            {
                prev_[2] = 0;
            }
            
            if (op->wrt==1 && prev_[3]==0)
            {
                //wheel retraction activated do buzz
                // when 1, wheels are retracting
                wheel_retraction_feedback(1);
                prev_[3] = 1;
            }
            else if (op->wrt==1 && prev_[3] == 1)
            {
                // when 0, wheels are deploying
                wheel_retraction_feedback(0);
                prev_[3] = 0;
            }
            if (op->start==1 && prev_[4]==0)
            {
                start_feedback();
                prev_[4] = 1;
            }
            else if (op->start ==1 && prev_[4] == 1)
            {
                prev_[4] = 0;
            }
            
            if (op->abort==1)
            {
                abort_feedback();
            }
            
        }
        void metCallback(const std_msgs::Float32::ConstPtr& MET)
        {
            nh_.param("MET",MET_END,MET_END); // need this here in case MET changes on the parameter server
            mqs_met = MET->data;
            if (mqs_met > MET_END)
            {
                met_feedback();
            }
        }

        /****** feedback functions here ******/
        void waterjet_feedback(float intensity)
        {
            sensor_msgs::JoyFeedbackArray feedback;
            sensor_msgs::JoyFeedback buzz;

            buzz.type = sensor_msgs::JoyFeedback::TYPE_RUMBLE;
            buzz.id = 1; // high freq rumble
            buzz.intensity = intensity * scale_feedback_; // use passed intensity value

            feedback.array.push_back(buzz);
            haptic_feedback_pub.publish(feedback);
        }

        void land_fwd_feedback(float intensity)
        {
            sensor_msgs::JoyFeedbackArray feedback;
            sensor_msgs::JoyFeedback buzz;

            buzz.type = sensor_msgs::JoyFeedback::TYPE_RUMBLE;
            buzz.id = 0; // low freq rumble
            buzz.intensity = 0.2 * intensity * scale_feedback_; // use passed intensity value

            feedback.array.push_back(buzz);
            haptic_feedback_pub.publish(feedback);
        }

        void land_rev_feedback(float intensity)
        {
            sensor_msgs::JoyFeedbackArray feedback;
            sensor_msgs::JoyFeedback buzz;

            buzz.type = sensor_msgs::JoyFeedback::TYPE_RUMBLE;
            buzz.id = 0; // mid freq rumble
            buzz.intensity = 0.2 * intensity * scale_feedback_; // since rev is limited do 20% max intensity

            feedback.array.push_back(buzz);
            haptic_feedback_pub.publish(feedback);
        }

        void sendPulse(double duration)
        {
            sensor_msgs::JoyFeedbackArray feedback;
            sensor_msgs::JoyFeedback buzz;

            buzz.type = sensor_msgs::JoyFeedback::TYPE_RUMBLE;
            buzz.id = 1; // high freq rumble
            buzz.intensity = scale_feedback_;

            feedback.array.push_back(buzz);
            haptic_feedback_pub.publish(feedback);

            ros::Duration(duration/1000.0).sleep(); // convert to milliseconds

            buzz.intensity = 0.0; // stop the rumble
            feedback.array.clear();
            feedback.array.push_back(buzz);
            haptic_feedback_pub.publish(feedback);
        }

        void wheel_retraction_feedback(int direction)
        {
            if(direction == 1)
            {
                changeIntensity(true,1.0,10);
            }
            else
            {
                changeIntensity(false,1.0,10);
            }
        }

        void abort_feedback()
        {
            std::string message = "CL"; // morse for closing station
            warnFeedback(message);
        }

        void start_feedback()
        {
            std::string message = "GG"; // morse for going
            warnFeedback(message);
        }
        void met_feedback()
        {
            std::string message = "R"; // morse for Received as transmitted
            warnFeedback(message);
        }

        void warnFeedback(const std::string& text)
        {
            // this function will translate the morese code to haptic feedback
            for (char c: text)
            {
                if (morseCodeMap.find(c) != morseCodeMap.end())
                {
                    std::string morseCode = morseCodeMap.at(c);
                    for (char mc : morseCode)
                    {
                        if (mc == '.')
                        {
                            sendPulse(100); // Dot, 100 ms
                        }
                        else if (mc == '-')
                        {
                            sendPulse(300); // Dash, 300 ms
                        }
                        ros::Duration(0.1).sleep(); // space between symbols
                        
                    }
                    ros::Duration(0.2).sleep(); // space between letters
                }
                else if (c == ' ')
                {
                    ros::Duration(0.7).sleep(); // space between words
                }
                
            }
        }

        void changeIntensity(bool increase, double duration, double rate)
        {
            const int steps = duration * rate;  // number of steps in scaling

            double intensity = increase ? 0.0:1.0;  // start from 0 or 1
            ros::Rate loop_rate(rate);

            sensor_msgs::JoyFeedbackArray feedback;
            sensor_msgs::JoyFeedback rumble;
            rumble.type = sensor_msgs::JoyFeedback::TYPE_RUMBLE;
            rumble.id = 1; // high freq
            for(int i =0;i<steps;++i)
            {
                if(increase)
                {
                    intensity = static_cast<double>(i)/static_cast<double>(steps);
                }
                else
                {
                    intensity = 1.0 - static_cast<double>(i)/static_cast<double>(steps);
                }
                rumble.intensity = intensity * scale_feedback_;
                feedback.array.clear();
                feedback.array.push_back(rumble);
                haptic_feedback_pub.publish(feedback);

                loop_rate.sleep();
            }

            // turn feedback off after loop completes
            rumble.intensity = 0.0;
            feedback.array.clear();
            feedback.array.push_back(rumble);
            haptic_feedback_pub.publish(feedback);
        }
        

};

const std::unordered_map<char, std::string> JoystickFeedback::morseCodeMap = {
    {'A', ".-"},
    {'B',"-..."},
    {'C',"-.-."},
    {'D',"-.."},
    {'E',"."},
    {'F',"..-."},
    {'G',"--."},
    {'H',"...."},
    {'I',".."},
    {'J',".---"},
    {'K',"-.-"},
    {'L',".-.."},
    {'M',"--"},
    {'N',"-."},
    {'O',"---"},
    {'P',".--."},
    {'Q',"--.-"},
    {'R',".-."},
    {'S',"..."},
    {'T',"-"},
    {'U',"..-"},
    {'V',"...-"},
    {'W',".--"},
    {'X',"-..-"},
    {'Y',"-.--"},
    {'Z',"--.."}
};

int main(int argc, char** argv)
{
    ros::init(argc,argv, "mqs_teleop_feedback");
    JoystickFeedback mqs_teleop_feedback; // I don't think the class instance name maters

    ros::spin();
    return 0;
}