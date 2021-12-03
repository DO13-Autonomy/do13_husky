#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <can_msgs/Frame.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>
#include <sstream>
#include <vector>
#include <bitset>


class FortJoyHandler
{
     public:
        // Declaring publishers and subscribers
        ros::Publisher joy_pub;
        ros::Subscriber can_sub;   
        ros::Publisher eStop_pub; 

        // Messages Intialization
        sensor_msgs::Joy joy_msg;                              
        // Header is the same header like the CAN message to keep the timestamp
        // axes = [left(x) left(y) right(x) right(y)]
        // buttones = [(1) (2) (3) (4) Left Up Right Down]

        // std_msgs::Bool eStop_msg;
        // eStop_msg = false;

        
        FortJoyHandler()
        {
            ros::NodeHandle n;
            
            joy_pub = n.advertise<sensor_msgs::Joy>("joy",1000);
            can_sub = n.subscribe<can_msgs::Frame>("/can_tx", 1000, &FortJoyHandler::can_callback, this);
            
        }
        
    private:
        // Declaring intial values       
        std::string NOT_SET = "00";
        std::string SET = "01";
        std::string ERROR = "10";
        std::string UNAVAILABLE = "11";
        int sign;

        void can_callback(can_msgs::Frame msg)
        {

            joy_msg.header = msg.header;
            joy_msg.axes.resize(4,0);
            joy_msg.buttons.resize(8,0);

            if (msg.id == 217962035)                                // Left Joystick - J1939 Basic Joystick Message 1
            {
                joy_msg.header.frame_id = "l";
                // Left X JoyStick      ==> Byte Offset = 0; Size = 2
                std::string binary_left_x = decemal_to_binary(msg.data[1]) + decemal_to_binary(msg.data[0]);
                unsigned long binary_left_x_magnitude = std::bitset<10>(binary_left_x.substr(0,10)).to_ulong();
                std::string binary_left_x_positive = binary_left_x.substr(10,2);
                std::string binary_left_x_negative = binary_left_x.substr(12,2);
                std::string binary_left_x_neutral = binary_left_x.substr(14,2);

                std::cout << binary_left_x << std::endl;

                if (binary_left_x_positive == SET) {sign = 1;}
                else if (binary_left_x_negative == SET) {sign = -1;}
                joy_msg.axes[0] = sign * float(binary_left_x_magnitude);

                // Left Y JoyStick      ==> Byte Offset = 2; Size = 2
                std::string binary_left_y = decemal_to_binary(msg.data[3]) + decemal_to_binary(msg.data[2]);
                unsigned long binary_left_y_magnitude = std::bitset<10>(binary_left_y.substr(0,10)).to_ulong();
                std::string binary_left_y_positive = binary_left_y.substr(10,2);
                std::string binary_left_y_negative = binary_left_y.substr(12,2);
                std::string binary_left_y_neutral = binary_left_y.substr(14,2);

                if (binary_left_y_positive == SET) {sign = 1;}
                else if (binary_left_y_negative == SET) {sign = -1;}
                joy_msg.axes[1] = sign * float(binary_left_y_magnitude);
                
                // Left Button Values   ==> Byte Offset = 5; Size = 1
                std::string binary_left_button = decemal_to_binary(msg.data[5]);
                std::string binary_left_left    = binary_left_button.substr(0,2);
                std::string binary_left_up      = binary_left_button.substr(2,2);
                std::string binary_left_right   = binary_left_button.substr(4,2);
                std::string binary_left_down    = binary_left_button.substr(6,2);

                joy_msg.buttons[4] = 0; joy_msg.buttons[5] = 0; joy_msg.buttons[6] = 0; joy_msg.buttons[7] = 0;
                if (binary_left_left == SET)    {joy_msg.buttons[4] = 1; joy_msg.buttons[5] = 0; joy_msg.buttons[6] = 0; joy_msg.buttons[7] = 0;}
                if (binary_left_up == SET)      {joy_msg.buttons[4] = 0; joy_msg.buttons[5] = 1; joy_msg.buttons[6] = 0; joy_msg.buttons[7] = 0;}
                if (binary_left_right == SET)   {joy_msg.buttons[4] = 0; joy_msg.buttons[5] = 0; joy_msg.buttons[6] = 1; joy_msg.buttons[7] = 0;}
                if (binary_left_down == SET)    {joy_msg.buttons[4] = 0; joy_msg.buttons[5] = 0; joy_msg.buttons[6] = 0; joy_msg.buttons[7] = 1;}

                joy_pub.publish(joy_msg);
            }
            else if (msg.id == 217962548)                           // Right Joystick - J1939 Basic Joystick Message 2
            {
                joy_msg.header.frame_id = "r";
                // right X JoyStick      ==> Byte Offset = 0; Size = 2
                std::string binary_right_x = decemal_to_binary(msg.data[1]) + decemal_to_binary(msg.data[0]);
                unsigned long binary_right_x_magnitude = std::bitset<10>(binary_right_x.substr(0,10)).to_ulong();
                std::string binary_right_x_positive = binary_right_x.substr(10,2);
                std::string binary_right_x_negative = binary_right_x.substr(12,2);
                std::string binary_right_x_neutral = binary_right_x.substr(14,2);

                if (binary_right_x_positive == SET) {sign = 1;}
                else if (binary_right_x_negative == SET) {sign = -1;}
                joy_msg.axes[2] = sign * float(binary_right_x_magnitude);

                // right Y JoyStick      ==> Byte Offset = 2; Size = 2
                std::string binary_right_y = decemal_to_binary(msg.data[3]) + decemal_to_binary(msg.data[2]);
                unsigned long binary_right_y_magnitude = std::bitset<10>(binary_right_y.substr(0,10)).to_ulong();
                std::string binary_right_y_positive = binary_right_y.substr(10,2);
                std::string binary_right_y_negative = binary_right_y.substr(12,2);
                std::string binary_right_y_neutral = binary_right_y.substr(14,2);

                if (binary_right_y_positive == SET) {sign = 1;}
                else if (binary_right_y_negative == SET) {sign = -1;}
                joy_msg.axes[3] = sign * float(binary_right_y_magnitude);

                // right Button Values   ==> Byte Offset = 5; Size = 1
                std::string binary_right_button = decemal_to_binary(msg.data[5]);
                std::string binary_right_one    = binary_right_button.substr(6,2);
                std::string binary_right_two    = binary_right_button.substr(4,2);
                std::string binary_right_three  = binary_right_button.substr(2,2);
                std::string binary_right_four   = binary_right_button.substr(0,2);

                joy_msg.buttons[0] = 0; joy_msg.buttons[1] = 0; joy_msg.buttons[2] = 0; joy_msg.buttons[3] = 0;
                if (binary_right_one == SET)    {joy_msg.buttons[0] = 1; joy_msg.buttons[1] = 0; joy_msg.buttons[2] = 0; joy_msg.buttons[3] = 0;}
                if (binary_right_two == SET)    {joy_msg.buttons[0] = 0; joy_msg.buttons[1] = 1; joy_msg.buttons[2] = 0; joy_msg.buttons[3] = 0;}
                if (binary_right_three == SET)  {joy_msg.buttons[0] = 0; joy_msg.buttons[1] = 0; joy_msg.buttons[2] = 1; joy_msg.buttons[3] = 0;}
                if (binary_right_four == SET)   {joy_msg.buttons[0] = 0; joy_msg.buttons[1] = 0; joy_msg.buttons[2] = 0; joy_msg.buttons[3] = 1;}

                joy_pub.publish(joy_msg);
            }
            else if (msg.id == 217966593)                           // Heartbeat - J1939 Custom Message
            {
                return;
            }
            else if (msg.id == 217966689)                           // Remote Status - J1939 Custom Message
            {
                return;
            }
            else {return;}
            
        }

        std::string decemal_to_binary(uint8_t byte)
        {
            std::string binary = std::bitset<8>(byte).to_string();            
            return binary;
        }

};


int main(int argc, char **argv)
{
    
    // EmergencyBrakeLidar(int roi_range_input, int threshold_input)


    ros::init(argc, argv, "emergency_stop_husky_gazebo");
    FortJoyHandler fjh;
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

