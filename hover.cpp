/** @file demo_flight_control.cpp
   *  @version 3.3
 *  @date September, 2017
  *
 *  @brief
 *  demo sample of how to use Local position control
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "dji_sdk_demo/hover.h"
#include "dji_sdk/dji_sdk.h"
#include <iostream>
#include <fstream>
ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
uint8_t current_gps_health = 0;
int num_targets = 0;
geometry_msgs::PointStamped local_position;
sensor_msgs::NavSatFix current_gps_position;
sensor_msgs::Imu curr_imu ;
geometry_msgs::Point initial_pos;
geometry_msgs::Point uwb_pos;
geometry_msgs::Point prev_pos;
geometry_msgs::Quaternion current_atti;

float kp=0.1;
float kd=0.01;
  std::ofstream imuAcc;
  std::ofstream imuG;
  std::ofstream imuRPY;
  std::ofstream uwb;
  std::ofstream comm;
  std::ofstream err;
ros::Publisher ctrlRollPitchYawHeightPub;

  ros::Publisher plotpos;

int main(int argc, char** argv)
{

 	imuAcc.open("acc4.txt");
 	imuG.open("gyro4.txt");


  ros::init(argc, argv, "demo_local_position_control_node");
  ros::NodeHandle nh;
  // Subscribe to messages from dji_sdk_node


  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  //ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
 // ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_callback);
//  ros::Subscriber gpsHealth      = nh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback);
  ros::Subscriber imu     =  nh.subscribe("/dji_sdk/imu",2,&imu_callback);
  
  //ros::Subscriber initialPosition = nh.subscribe("initial_pos", 1, &initial_pos_callback);

  // Publish the control signal
   ros::Subscriber uwbPosition = nh.subscribe("uwb_pos", 1, &uwb_position_callback);
  ctrlRollPitchYawHeightPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 1);
  
// Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;
  imuAcc.open("acc4.txt");
 	imuG.open("gyro4.txt");

  
    ROS_INFO("M100 taking off!");
  //  takeoff_result = M100monitoredTakeoff();
    if(takeoff_result){
      printf("READY TO CONTROL \n");
      target_set_state = 1;
    }

  ros::spin();
  return 0;
}

/*!
 * This function is called when local position data is available.
 * In the example below, we make use of two arbitrary targets as
 * an example for local position control.
 *
 */
void uwb_position_callback(const geometry_msgs::Point::ConstPtr& msg) {
  
  uwb_pos = *msg;
  e1 = ros::Time::now();
  dt1= (end_ - start_).toNSec() * 1e-9;
  s1 = ros::Time::now();
   //max pitch and roll angle 0.1 rad

  //-1 pitch opposite sign
  dx_error=(0.0 - (uwb_pos.x-prev_pos.x));
  dy_error=(0.0 - (uwb_pos.y-prev_pos.y) ); 
  x_error =(5.0- uwb_pos.x);
  y_error = 2.2 - uwb_pos.y;
 // dximu+=(uwb_pos.x-prev_pos.x);
 // dyimu+=(uwb_pos.y-prev_pos.y);
  
 // uwb<<uwb_pos.x<<" "<<uwb_pos.y<<dt1<<std::endl;//" "<<dximu<<" "<<dyimu<<" "<<dt1<<std::endl; 
  //err<<x_error<<" "<<y_error<<std::endl;




  //calculate desired roll and pitch
  pitch=kp*x_error+kd*dx_error;
  roll=kp*y_error+kd*dy_error;
 // double yawInRad= toEulerAngle(current_atti).z;  
  
  //printf("Yaw %f degree",yawInRad*rad2deg); 
  //printf("INITIAL x %f  y %f \n",initial_pos.x,initial_pos.y);
 //printf(" ex= %f  ey= %f   \n",x_error,y_error );
  //printf("UWB x %f  y %f \n",uwb_pos.x,uwb_pos.y);
  if (fabs(pitch) >= max_angle)

    pitchd = (pitch>0) ? max_angle : -1 * max_angle;
  else
    pitchd = pitch;

  if (fabs(roll) >= max_angle)
    rolld = (roll>0) ? max_angle : -1 * max_angle;
  else
    rolld = roll;

   
  //comm<<pitch<<" "<<pitchd<<" "<<roll<<" "<<rolld<<std::endl;
  //float rolld=cos(yawInRad)*roll-sin(yawInRad)*pitch;
  //float pitchd=sin(yawInRad)*roll+cos(yawInRad)*pitch;
  sensor_msgs::Joy controlmsg;
  controlmsg.axes.push_back(0);
  controlmsg.axes.push_back(pitchd);
  controlmsg.axes.push_back(1.5);
  controlmsg.axes.push_back(0);
  //ctrlRollPitchYawHeightPub.publish(controlmsg);
  prev_pos=uwb_pos;
}


void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  curr_imu=*msg;
  end_ = ros::Time::now();
  dt= (end_ - start_).toNSec() * 1e-9; //sec
  start_ = ros::Time::now();
  
  imuAcc<<curr_imu.linear_acceleration.x<<" "<<curr_imu.linear_acceleration.y<<" "<<curr_imu.linear_acceleration.z<<" "<<dt<<std::endl;
  imuG<<curr_imu.angular_velocity.x<<" "<<curr_imu.angular_velocity.y<<" "<<curr_imu.angular_velocity.z<<std::endl;
 // xspeed+=((curr_imu.linear_acceleration.x+ax_prev)/2.0)*dt;
 // yspeed+=((curr_imu.linear_acceleration.y+ay_prev)/2.0)*dt;

//  dximu+=((xspeed_prev+xspeed)/2.0)*dt;  
 // dyimu+=((yspeed_prev+yspeed)/2.0)*dt; 
 // imuAcc<<dximu<<" "<<dyimu<<" "<<dt<<" "<<xspeed<<" "<<yspeed<<std::endl;
//imuAcc<<curr_imu.linear_acceleration.x<<" "<<curr_imu.linear_acceleration.y<<" "<<curr_imu.linear_acceleration.z<<" "<<dt<<std::endl;
  //imuG<<curr_imu.angular_velocity.x<<" "<<curr_imu.angular_velocity.y<<" "<<curr_imu.angular_velocity.z<<std::endl;

  

}


/*
void initial_pos_callback(const geometry_msgs::Point::ConstPtr& msg)
{
  initial_pos=*msg;
}
*/


void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}





bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}



/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */

 
bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps_position.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR )
  {
    ROS_INFO("Takeoff failed.");
    //return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }
  return true;
}
