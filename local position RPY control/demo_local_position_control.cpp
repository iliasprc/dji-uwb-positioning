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

#include "dji_sdk_demo/demo_local_position_control.h"
#include "dji_sdk/dji_sdk.h"
#include "dji_sdk_demo/Pos.h"
#include <fstream>
#include <chrono>
#include <string>

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
geometry_msgs::PointStamped prev_position;
sensor_msgs::NavSatFix current_gps_position;
dji_sdk_demo::Pos uwb_pos;
geometry_msgs::Quaternion current_atti;
 sensor_msgs::Imu curr_imu ;
  std::ofstream uwb;
  std::ofstream gpspos;
  std::ofstream err;

  std::ofstream imuAcc;
  std::ofstream imuG;
  std::ofstream imuRPY;

float max_angle=0.1;
float kp=0.05;
float ki=0.001;
float kd=0.01;

float bi,ad,bd,br;
float Tf=0.1;float Tt;

float dx,dy,dx2,dy2;
float ix,iy,ix2,iy2;

float ex,ey;float ex2,ey2;
geometry_msgs::Vector3 rpy;
  using namespace std::chrono;
high_resolution_clock::time_point t1;
high_resolution_clock::time_point t2;
high_resolution_clock::time_point t3;
high_resolution_clock::time_point t4;
int main(int argc, char** argv)
{


  uwb.open("dataflight/uwbpos.txt");
  gpspos.open("dataflight/gpspos.txt");
  //err.open("dataflight/derrors1.txt");

  ros::init(argc, argv, "demo_local_position_control_node");
  ros::NodeHandle nh;
  // Subscribe to messages from dji_sdk_node
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_callback);
  ros::Subscriber gpsHealth      = nh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback);
  ros::Subscriber uwbPosition = nh.subscribe("uwb_pos", 1, &uwb_position_callback);
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber imu     =  nh.subscribe("/dji_sdk/imu", 10 ,&imu_callback);
  // Publish the control signal
  // ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 1);
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 1);
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;
  if (!set_local_position())
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }

  if(is_M100())
  {
    ROS_INFO("M100 taking off!");
    takeoff_result = M100monitoredTakeoff();
  }
  else
  {
    ROS_INFO("A3/N3 taking off!");
    takeoff_result = monitoredTakeoff();
  }

  if(takeoff_result)
  {
    //! Enter total number of Targets
    num_targets = 2;
    //! Start Mission by setting Target state to 1
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
void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  local_position = *msg;
  double xCmd, yCmd, zCmd;
  sensor_msgs::Joy controlPosYaw;

  // Down sampled to 50Hz loop
  if (elapsed_time > ros::Duration(0.02)) {
    start_time = ros::Time::now();
    if (target_set_state == 1) {
      //! First arbitrary target
      if (current_gps_health > 3) {
        setTarget(1, 1, 1.5, 0);
        local_position_ctrl(xCmd, yCmd, zCmd);
      }
      else
      {
        ROS_INFO("Cannot execute Local Position Control");
        ROS_INFO("Not enough GPS Satellites");
        //! Set Target set state to 0 in order to stop Local position control mission
        target_set_state = 0;
      }
    }

    if (target_set_state == 3) {
      //! Second arbitrary target
      if (current_gps_health > 3) {
        setTarget(0, 0, 0, 0);
        local_position_ctrl(xCmd, yCmd, zCmd);
      }
      else
      {
        ROS_INFO("Cannot execute Local Position Control");
        ROS_INFO("Not enough GPS Satellites");
        //! Set Target set state to 0 in order to stop Local position control mission
        target_set_state = 0;
      }
    }
  }
}

void uwb_position_callback(const dji_sdk_demo::Pos::ConstPtr& msg) {

uwb_pos=*msg;

uwb<<uwb_pos.x<<" "<<uwb_pos.y<<" "<<uwb_pos.z<<uwb_pos.nses<<std::endl;
gpspos<<local_position.point.x<<" "<<local_position.point.y<<" "<<local_position.point.z<<std::endl;

}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  curr_imu=*msg;
  
    t1 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t1 - t2);
   
  //  auto duration = duration_cast<microseconds>( t2 - t1 ).count()
   
  
imuAcc<<curr_imu.linear_acceleration.x<<" "<<curr_imu.linear_acceleration.y<<" "<<curr_imu.linear_acceleration.z<<" "<<time_span.count()<<std::endl;
imuG<<curr_imu.angular_velocity.x<<" "<<curr_imu.angular_velocity.y<<" "<<curr_imu.angular_velocity.z<<std::endl;
t2 = high_resolution_clock::now();

}




void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  current_gps_position = *msg;
}

void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg) {
  current_gps_health = msg->data;
}

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
  rpy=toEulerAngle(current_atti);
  imuRPY<<rpy.x<<" "<<rpy.y<<" "<<rpy.z<<std::endl;
}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}



/*!


/*!
 * This function calculates the difference between target and current local position
 * and sends the commands to the Position and Yaw control topic.
 *
 */
void local_position_ctrl(double &xCmd, double &yCmd, double &zCmd)
{
  
  ad=Tf/(Tf+0.02);
  bd=kd/(Tf+0.02);
  bi=ki*0.02;
  br=0.02/0.1;
	//// VELOCITY FORM


  zCmd = target_offset_z;
  dx=ad*dx2-bd*(local_position.point.x-prev_position.point.x);
  dy=ad*dy2-bd*(local_position.point.y-prev_position.point.y);
  dx2=dx;
  dy2=dy;
    roll= kp*(target_offset_x - local_position.point.x)+dx+ix;
  pitch = pitch+ kp*(target_offset_y - local_position.point.y)+dy+iy;
  ex2=ex;
  ey2=ey;
  ex=target_offset_x - local_position.point.x;
  ey=target_offset_y - local_position.point.y;
 // pitch=cos(rpy.z)*xCmd-sin(rpy.z)*yCmd;
  //std::cout<<" x "<<local_position.point.x<<"  y "<<local_position.point.y<<std::endl;
   // roll=sin(rpy.z)*xCmd+cos(rpy.z)*yCmd;
    if (fabs(pitch) >= max_angle)

    pitchd = (pitch>0) ? max_angle : -1 * max_angle;
  else
    pitchd = pitch;

  if (fabs(roll) >= max_angle)
    rolld = (roll>0) ? max_angle : -1 * max_angle;
  else
    rolld = roll;

  ix=ix2+bi*(target_offset_x - local_position.point.x)+br*(rolld-roll);
   iy=iy2+bi*(target_offset_y - local_position.point.y)+br*(pitchd-pitch);
   prev_position.point.x=local_position.point.x;
   prev_position.point.y=local_position.point.y;
  sensor_msgs::Joy controlPosYaw;
  controlPosYaw.axes.push_back(0);
  controlPosYaw.axes.push_back(pitchd);
  controlPosYaw.axes.push_back(zCmd);
  controlPosYaw.axes.push_back(0);
  ctrlPosYawPub.publish(controlPosYaw);

  // 0.1m or 10cms is the minimum error to reach target in x y and z axes.
  // This error threshold will have to change depending on aircraft/payload/wind conditions.
  /*
  if (((std::abs(xCmd)) < 0.1) && ((std::abs(yCmd)) < 0.1) &&
      (local_position.point.z > (target_offset_z - 0.1)) && (local_position.point.z < (target_offset_z + 0.1))) {
    if(target_set_state <= num_targets) {
      ROS_INFO("%d of %d target(s) complete", target_set_state, num_targets);
      target_set_state++;
    }
    else
    {
      target_set_state = 0;
    }
  }
  */
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

/*
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
         (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
         ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}


/*!
 * This function demos how to use M100 flight_status
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

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
     current_gps_position.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }
  return true;
}

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);

  return (bool)localPosReferenceSetter.response.result;
}
