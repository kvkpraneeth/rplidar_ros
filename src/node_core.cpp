#include "hal/types.h"
#include "ros/duration.h"
#include "ros/forwards.h"
#include "ros/node_handle.h"
#include "rplidar_driver.h"
#include "rplidar_ros/node.h"
#include <stdexcept>

void rplidar_ros::publish_scan(rplidar_response_measurement_node_hq_t *nodes, size_t node_count, float angle_min, float angle_max)
{
    static int scan_count = 0;

    this->scanPub->msg_.header.stamp = this->start_scan_time;
    this->scanPub->msg_.header.frame_id = this->frame_id;

    scan_count++;
    
    bool reversed = (angle_max > angle_min);
    if ( reversed ) {
      scanPub->msg_.angle_min =  M_PI - angle_max;
      scanPub->msg_.angle_max =  M_PI - angle_min;
    } else {
      scanPub->msg_.angle_min =  M_PI - angle_min;
      scanPub->msg_.angle_max =  M_PI - angle_max;
    }
    scanPub->msg_.angle_increment =
        (scanPub->msg_.angle_max - scanPub->msg_.angle_min) / (double)(node_count-1);

    scanPub->msg_.scan_time = scan_duration;
    scanPub->msg_.time_increment = scan_duration / (double)(node_count-1);
    scanPub->msg_.range_min = 0.15;
    scanPub->msg_.range_max = max_distance;//8.0;

    scanPub->msg_.intensities.resize(node_count);
    scanPub->msg_.ranges.resize(node_count);
    bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
    if (!reverse_data) {
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float) nodes[i].dist_mm_q2/4.0f/1000;
            if (read_value == 0.0)
                scanPub->msg_.ranges[i] = std::numeric_limits<float>::infinity();
            else
                scanPub->msg_.ranges[i] = read_value;
            scanPub->msg_.intensities[i] = (float) (nodes[i].quality >> 2);
        }
    } else {
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float)nodes[i].dist_mm_q2/4.0f/1000;
            if (read_value == 0.0)
                scanPub->msg_.ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
            else
                scanPub->msg_.ranges[node_count-1-i] = read_value;
            scanPub->msg_.intensities[node_count-1-i] = (float) (nodes[i].quality >> 2);
        }
    }

    this->scanPub->unlockAndPublish();
}

bool rplidar_ros::getRPLIDARDeviceInfo()
{
    u_result opl_result;

    rplidar_response_device_info_t devinfo;

    opl_result = drv->getDeviceInfo(devinfo);
    if (IS_FAIL(opl_result)) {
        if (opl_result == RESULT_OPERATION_TIMEOUT) {
            ROS_ERROR("Error, operation time out. RESULT_OPERATION_TIMEOUT! ");
        } else {
            ROS_ERROR("Error, unexpected error, code: %x",opl_result);
        }
        return false;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }
    printf("\n");
    ROS_INFO("Firmware Ver: %d.%02d",devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
    ROS_INFO("Hardware Rev: %d",(int)devinfo.hardware_version);
    return true;
}


bool rplidar_ros::checkRPLIDARHealth()
{
    u_result opl_result;
    rplidar_response_device_health_t healthinfo;

    opl_result = drv->getHealth(healthinfo);
    if (IS_OK(opl_result)) { 
        ROS_INFO("RPLidar health status : %d", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            ROS_ERROR("Error, rplidar internal error detected. Please reboot the device to retry.");
            return false;
        } else {
            return true;
        }

    } else {
        ROS_ERROR("Error, cannot retrieve rplidar health code: %x", opl_result);
        return false;
    }
}

bool rplidar_ros::stop_motor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  if(!drv)
       return false;

  ROS_DEBUG("Stop motor");
  drv->stop();
  drv->stopMotor();
  return true;
}

bool rplidar_ros::start_motor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  if(!drv)
       return false;
  ROS_DEBUG("Start motor");
  drv->startMotor();
  drv->startScan(0,1);
  return true;
}

float rplidar_ros::getAngle(const rplidar_response_measurement_node_hq_t& node)
{
    return node.angle_z_q14 * 90.f / 16384.f;
}

rplidar_ros::rplidar_ros(ros::NodeHandle& nh, RPlidarDriver* drv)
{
    this->nnh = nh;
    
    //Get Required Parameters
    nnh.param<std::string>("serial_port", this->serial_port, "/dev/ttyUSB0");
    nnh.param<int>("serial_baudrate", serial_baudrate, 115200/*256000*/);//ros run for A1 A2, change to 256000 if A3
    nnh.param<std::string>("frame_id", frame_id, "laser_frame");
    nnh.param<bool>("inverted", inverted, false);
    nnh.param<bool>("angle_compensate", angle_compensate, false);
    nnh.param<std::string>("scan_mode", scan_mode, std::string());

    this->drv = drv;
    
    if (!drv) {
        ROS_ERROR("Create Driver fail, exit");
        std::runtime_error("");
    }

    // make connection...
    if (IS_FAIL(drv->connect(serial_port.c_str(), (_u32)serial_baudrate))) {
        ROS_ERROR("Error, cannot bind to the specified serial port %s.",serial_port.c_str());
        RPlidarDriver::DisposeDriver(drv);
        std::runtime_error("");
    }

    // get rplidar device info
    if (!getRPLIDARDeviceInfo()) {
        std::runtime_error("Device Info not recieved. Exiting...");
    }

    // check health...
    if (!checkRPLIDARHealth()) {
        RPlidarDriver::DisposeDriver(drv);
        std::runtime_error("Lidar Health issues. Exiting...");
    }
  
 
    this->stop_motor_service = nh.advertiseService("stop_motor", &rplidar_ros::stop_motor, this);
    this->start_motor_service = nh.advertiseService("start_motor", &rplidar_ros::start_motor, this);

    this->scanPub.reset();

    this->drv->startMotor(); 

    RplidarScanMode current_scan_mode;
    
    if (scan_mode.empty()) {
        op_result = drv->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);
    } else {
        std::vector<RplidarScanMode> allSupportedScanModes;
        op_result = drv->getAllSupportedScanModes(allSupportedScanModes);

        if (IS_OK(op_result)) {
            _u16 selectedScanMode = _u16(-1);
            for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                if (iter->scan_mode == scan_mode) {
                    selectedScanMode = iter->id;
                    break;
                }
            }

            if (selectedScanMode == _u16(-1)) {
                ROS_ERROR("scan mode `%s' is not supported by lidar, supported modes:", scan_mode.c_str());
                for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                    ROS_ERROR("\t%s: max_distance: %.1f m, Point number: %.1fK",  iter->scan_mode,
                            iter->max_distance, (1000/iter->us_per_sample));
                }
                op_result = RESULT_OPERATION_FAIL;
            } else {
                op_result = drv->startScanExpress(false /* not force scan */, selectedScanMode, 0, &current_scan_mode);
            }
        }
    }

    if(IS_OK(op_result))
    {
        //default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us
        angle_compensate_multiple = (int)(1000*1000/current_scan_mode.us_per_sample/10.0/360.0);
        if(angle_compensate_multiple < 1) 
          angle_compensate_multiple = 1;
        max_distance = current_scan_mode.max_distance;
        ROS_INFO("current scan mode: %s, max_distance: %.1f m, Point number: %.1fK , angle_compensate: %d",  current_scan_mode.scan_mode,
                 current_scan_mode.max_distance, (1000/current_scan_mode.us_per_sample), angle_compensate_multiple);
    }
    else
    {
        ROS_ERROR("Can not start scan: %08x!", op_result);
    }


}

void rplidar_ros::loop(ros::TimerEvent& event)
{
        rplidar_response_measurement_node_hq_t nodes[360*8];
        size_t   count = _countof(nodes);

        start_scan_time = ros::Time::now();
        op_result = drv->grabScanDataHq(nodes, count);
        end_scan_time = ros::Time::now();
        scan_duration = (end_scan_time - start_scan_time).toSec();

        if (op_result == RESULT_OK) {
            op_result = drv->ascendScanData(nodes, count);
            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(359.0f);
            if (op_result == RESULT_OK) {
                if (angle_compensate) {
                    //const int angle_compensate_multiple = 1;
                    const int angle_compensate_nodes_count = 360*angle_compensate_multiple;
                    int angle_compensate_offset = 0;
                    rplidar_response_measurement_node_hq_t angle_compensate_nodes[angle_compensate_nodes_count];
                    memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(rplidar_response_measurement_node_hq_t));

                    int i = 0, j = 0;
                    for( ; i < count; i++ ) {
                        if (nodes[i].dist_mm_q2 != 0) {
                            float angle = getAngle(nodes[i]);
                            int angle_value = (int)(angle * angle_compensate_multiple);
                            if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                            for (j = 0; j < angle_compensate_multiple; j++) {
                                angle_compensate_nodes[angle_value-angle_compensate_offset+j] = nodes[i];
                            }
                        }
                    }

                    publish_scan(angle_compensate_nodes, angle_compensate_nodes_count, angle_min, angle_max);
                

                } else {
                    int start_node = 0, end_node = 0;
                    int i = 0;
                    // find the first valid node and last valid node
                    while (nodes[i++].dist_mm_q2 == 0);
                    start_node = i-1;
                    i = count -1;
                    while (nodes[i--].dist_mm_q2 == 0);
                    end_node = i+1;

                    angle_min = DEG2RAD(getAngle(nodes[start_node]));
                    angle_max = DEG2RAD(getAngle(nodes[end_node]));

                    publish_scan(&nodes[start_node], end_node-start_node+1, angle_min, angle_max);
               }
            } else if (op_result == RESULT_OPERATION_FAIL) {
                // All the data is invalid, just publish them
                float angle_min = DEG2RAD(0.0f);
                float angle_max = DEG2RAD(359.0f);

                publish_scan(nodes, count, angle_min, angle_max);
            }
        }
}
