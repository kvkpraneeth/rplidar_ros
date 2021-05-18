#ifndef NODE_H
#define NODE_H

#include "hal/types.h"
#include "ros/forwards.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "ros/service_server.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "rplidar.h"
#include "realtime_tools/realtime_publisher.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace rp::standalone::rplidar;

class rplidar_ros
{
    
    //Driver Instances of the node.
    private: 

        RPlidarDriver* drv;

        u_result op_result;

        RplidarScanMode current_scan_mode;

    //Worker Functions.
    public:

        void publish_scan(rplidar_response_measurement_node_hq_t *nodes, size_t node_count, float angle_min, float angle_max);

        bool getRPLIDARDeviceInfo();

        bool checkRPLIDARHealth();

        bool stop_motor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

        bool start_motor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

        static float getAngle(const rplidar_response_measurement_node_hq_t& node);

        rplidar_ros(ros::NodeHandle& nh, RPlidarDriver* drv);

    //Constants of the lidar.
    private:

        std::string serial_port;

        int serial_baudrate;
        
        std::string frame_id;
        
        bool inverted;
        
        bool angle_compensate;

        float max_distance;

        int angle_compensate_multiple;

        std::string scan_mode;

    // ROS instances of the node.
    private:
        
        std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::LaserScan>> scanPub;
        
        ros::NodeHandle nnh;

    public: ros::Timer loopTimer;

    public: void loop(ros::TimerEvent &event);

    private: 

        ros::ServiceServer stop_motor_service;

        ros::ServiceServer start_motor_service;

    //Other Instances.
    private:
        
        ros::Time start_scan_time, end_scan_time;

        double scan_duration; 

};

#endif
