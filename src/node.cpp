
#include "rplidar_ros/node.h"
#include "ros/duration.h"
#include "ros/node_handle.h"
#include "ros/spinner.h"

RPlidarDriver * drv = NULL;

int main(int argc, char * argv[]) {

    ros::init(argc, argv, "rplidar_node");

    u_result op_result;

    ros::NodeHandle mnh;

    ros::AsyncSpinner spinner(0);

    // create the driver instance
    drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
    
    rplidar_ros* r;

    *r = rplidar_ros(mnh, drv);

    r->loopTimer = mnh.createTimer(ros::Duration(0.1), &rplidar_ros::loop, r);

    spinner.start();

    drv->stop();
    
    drv->stopMotor();

    RPlidarDriver::DisposeDriver(drv);
    
    mnh.shutdown();
}
