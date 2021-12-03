#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <can_msgs/Frame.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>
#include <sstream>
#include <vector>
#include <geometry_msgs/Twist.h>

class TeleOpPubulisher
{
     public:
        // Declaring publishers and subscribers
        ros::Publisher cmdVel_pub;
        ros::Subscriber joy_sub;   

        // Messages Intialization
        geometry_msgs::Twist twist_msg;

        TeleOpPubulisher()
        {
            ros::NodeHandle n;
            
            cmdVel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
            joy_sub = n.subscribe<sensor_msgs::Joy>("/joy", 1000, &TeleOpPubulisher::joy_callback, this);
        }
        
    private:
        // Declaring intial values       
        int joy_axe_max = 1021;
        int cmdVel_max = 5;



        void joy_callback(sensor_msgs::Joy msg)
        {
            // axes = [left(x) left(y) right(x) right(y)]
            // buttones = [(1) (2) (3) (4) Left Up Right Down]


                twist_msg.linear.x  = msg.axes[1] / joy_axe_max * cmdVel_max;           // left controls the speed
                twist_msg.angular.z  = -msg.axes[3] / joy_axe_max * cmdVel_max;         // right controls rotation

                cmdVel_pub.publish(twist_msg);
           
            
        }



};


int main(int argc, char **argv)
{
    
    // EmergencyBrakeLidar(int roi_range_input, int threshold_input)


    ros::init(argc, argv, "TeleOpPubulisher");
    TeleOpPubulisher tp;
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

