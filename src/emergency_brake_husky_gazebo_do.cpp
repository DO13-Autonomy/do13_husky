#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <vector>
#include <raptor_dbw_msgs/BrakeCmd.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>


#define M_PI 3.14159265358979323846


class EmergencyBrakeLidar
{
     public:
        // Declaring publishers and subscribers
        ros::Publisher roi_pub;
        ros::Publisher emergency_cmd_vel_pub;
        ros::Subscriber lidar_sub;    
        
        EmergencyBrakeLidar(int roi_range_input, int threshold_input)
        {
            ros::NodeHandle n;
            
            roi_pub = n.advertise<sensor_msgs::LaserScan>("emergency/scan_roi", 1000);
            emergency_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("emergency/cmd_vel",1000);
            lidar_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &EmergencyBrakeLidar::lidar_callback, this);
            
            setROI_range(roi_range_input);
            setThreshold(threshold_input);
        }
        
    private:
        // Declaring intial values       
        double front_distance_intialization = INFINITY;
        int Threshold;                                //threshhold to activate emergency brake
        int ROI_range;

        int setROI_range(int roi_range_input)
        {
            ROI_range = roi_range_input;
            return ROI_range;
        }

        int setThreshold(int threshold_input)
        {
            Threshold = threshold_input;
            return Threshold;
        }

        void lidar_callback(sensor_msgs::LaserScan msg)
        {
            // Declaring scan messages
            sensor_msgs::LaserScan input_scan;
            sensor_msgs::LaserScan roi_scan;
            input_scan = msg;


            // Lidar properties 
            float lidar_angle_max = input_scan.angle_max;
            float lidar_angle_min = input_scan.angle_min;
            float lidar_angle_increment = input_scan.angle_increment;

            // lidar range calculations (assuming the lidar is mounted symetrically)
            if (abs(lidar_angle_max) != abs(lidar_angle_min))
            {
                // std::cout << "Error: Lidar readings are not symetric, please review the calculations" << std::endl;
                ROS_ERROR("Error: Lidar readings are not symetric, please review the calculations");
                exit(0);
            } 
            int lidar_points_count = round(abs(lidar_angle_max)*2/lidar_angle_increment) + 1;
            int min_range = (lidar_points_count/2) - round((ROI_range*M_PI/180)/lidar_angle_increment);                                      // starting index of the ROI
            int max_range = (lidar_points_count/2) + round((ROI_range*M_PI/180)/lidar_angle_increment);;                                     // max index of the ROI

            double front_min_distance = front_distance_intialization;                          //setting the intial threshold high enough every loop

            //the main loop for (1) data ROI filtering and (2) actuation 
            for (int i=min_range; i<=max_range; i++)
            {        
                // adding the points from ROI to the new point cloud of ROI
                roi_scan.ranges.push_back(input_scan.ranges[i]);    
            
                // estimating the closest point in the front of the vehicle
                if (input_scan.ranges[i] < front_min_distance)
                {front_min_distance = input_scan.ranges[i] ;}
            }
            
            // publishing the Region of Interest of the lidar for dignoses
            roi_scan.header = input_scan.header;
            roi_scan.angle_increment = input_scan.angle_increment;
            roi_scan.angle_max = input_scan.angle_max;
            roi_scan.angle_min = input_scan.angle_min;
            roi_scan.range_max = input_scan.range_max;
            roi_scan.range_min = input_scan.range_min;
            roi_scan.scan_time = input_scan.scan_time;
            roi_pub.publish(roi_scan); 
    
            EmergencyBrakeLidar::cmd_vel(front_min_distance);

            //printing out the min value detected
            // ROS_DEBUG("%f", front_min_distance);
            std::cout << front_min_distance << std::endl; 

        }

        void cmd_vel(double front_min_distance)
        {
            geometry_msgs::Twist cmd_vel;

            if (front_min_distance < Threshold)
            {
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.linear.z = 0;
                cmd_vel.angular.x = 0;
                cmd_vel.angular.y = 0;
                cmd_vel.angular.z = 0;

                emergency_cmd_vel_pub.publish(cmd_vel);
            }
                        
        }  
};


int main(int argc, char **argv)
{
    
    // EmergencyBrakeLidar(int roi_range_input, int threshold_input)

    if (argc != 3)
    {
        ROS_ERROR("Error: missing input. Two arguments are needed [Lidar Region of Interest in Degrees] [Distance threshold to brale]");
        exit(0);
    }

    ros::init(argc, argv, "emergency_stop_husky_gazebo");
    EmergencyBrakeLidar emergency_brake_lidar = EmergencyBrakeLidar(atoi(argv[1]), atoi(argv[2]));
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

