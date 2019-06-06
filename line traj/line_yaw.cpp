 /** @file hover.cpp

  Ilias Papastratis
 */

#include "dji_sdk_demo/line.h"

#include "dji_sdk/dji_sdk.h"

#include <chrono>


ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;



float kp=0.1;
float ki=0.01;
float kd=4.0;
float max_angle=0.1;

ros::Publisher ctrlRollPitchYawHeightPub;

  ros::Publisher plotpos;
  using namespace std::chrono;
high_resolution_clock::time_point t1;
high_resolution_clock::time_point t2;
high_resolution_clock::time_point t3;
high_resolution_clock::time_point t4;

int main(int argc, char** argv)
{
   //desired_pos.x=5;
  // desired_pos.y=2;
  // t2 = high_resolution_clock::now();
  // t4 = high_resolution_clock::now();
  ros::init(argc, argv, "demo_local_position_control_node");
  ros::NodeHandle nh;
  // Subscribe to messages from dji_sdk_node

 
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  //::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
 // ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
 // ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_callback);
 // ros::Subscriber gpsHealth      = nh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback);
  ros::Subscriber imu     =  nh.subscribe("/dji_sdk/imu",2,&imu_callback);
  
  //ros::Subscriber initialPosition = nh.subscribe("initial_pos", 1, &initial_pos_callback);

  // Publish the control signal
  
// Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;


   /*
  //imuAcc.open("distacc.txt");
 //	imuG.open("accels.txt");
  uwb.open("velpos30.txt");
  comm.open("velcomms30.txt");
  err.open("pid30.txt");
  err<<kp<<" "<<ki<<" "<<kd<<" "<<std::endl;
 */
   fileid="k1";
   open_data_files(fileid);

    ROS_INFO("M100 taking off!");
    t1 = high_resolution_clock::now();
   t2 = high_resolution_clock::now();
  ctrlRollPitchYawHeightPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 1);
   ros::Subscriber uwbPosition = nh.subscribe("uwb_pos", 1, &uwb_position_callback);
 

   takeoff_result = M100monitoredTakeoff();

  // t2 = high_resolution_clock::now();
   //t4 = high_resolution_clock::now();
   if(takeoff_result){
      printf("READY TO CONTROL \n");
   }
   //ctrlRollPitchYawHeightPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 1);
   //ros::Subscriber uwbPosition = nh.subscribe("uwb_pos", 1, &uwb_position_callback);
   printf(" Going to state 0 (3 1,5)\n" );
  ros::spin();
  return 0;
}  

/*!
 * This function is called when local position data is available.
 * In the example below, we make use of two arbitrary targets as
 * an example for local position control.
 *
 */


geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

void pid_pos_form(){

  if((desired_pos.x-curr_pos.x)>0.5||(desired_pos.x-curr_pos.x)>0.5){
  	max_angle=0.09;
  }
  else {
  	max_angle=0.15;
  }
  pitch=kp*(desired_pos.x-curr_pos.x)+kd*(desired_pos.x-curr_pos.x-x_error)-0.005;//angle offsset due to imu calibration
  roll=kp*(desired_pos.y-curr_pos.y)+kd*(desired_pos.y-curr_pos.y-y_error);
  
  if (fabs(pitch) >= max_angle)

    pitchdes = (pitch>0) ? max_angle : -1 * max_angle;
  else
    pitchdes = pitch;

  if (fabs(roll) >= max_angle)
    rolldes = (roll>0) ? max_angle : -1 * max_angle;
  else
    rolldes = roll;


  uint8_t flag = (DJISDK::VERTICAL_POSITION   |
                DJISDK::HORIZONTAL_ANGLE |
                DJISDK::YAW_ANGLE            |
                DJISDK::HORIZONTAL_BODY  |
                DJISDK::STABLE_ENABLE);
  sensor_msgs::Joy controlmsg;
  controlmsg.axes.push_back(rolldes);
  controlmsg.axes.push_back(pitchdes);
  controlmsg.axes.push_back(1.2);
  controlmsg.axes.push_back(0);
  controlmsg.axes.push_back(flag);
  ctrlRollPitchYawHeightPub.publish(controlmsg);

 // comm<<pitch<<" "<<pitchd<<" "<<roll<<" "<<rolld<<std::endl ;

  x_error=desired_pos.x-curr_pos.x;
  y_error=desired_pos.y-curr_pos.y;

}




void pid_vel_form(){


  x_error=desired_pos.x-curr_pos.x;
  y_error=desired_pos.y-curr_pos.y;


  pitch=pitchdes+kp*(x_error-x_error1)+ki*x_error+kd*(x_error-2*x_error1+x_error2);
  roll=rolldes+kp*(y_error-y_error1)+ki*y_error+kd*(y_error-2*y_error1+y_error2);
  

  if (fabs(pitch) >= max_angle)

    pitchdes = (pitch>0) ? max_angle : -1 * max_angle;
  else
    pitchdes = pitch;

  if (fabs(roll) >= max_angle)
    rolldes = (roll>0) ? max_angle : -1 * max_angle;
  else
    rolldes = roll;

  sensor_msgs::Joy controlmsg;
  controlmsg.axes.push_back(rolldes);
  controlmsg.axes.push_back(pitchdes);
  controlmsg.axes.push_back(1.2);
  controlmsg.axes.push_back(0);
  ctrlRollPitchYawHeightPub.publish(controlmsg);

  

  x_error2=x_error1;
  x_error1=x_error;
  y_error2=y_error1;
  y_error1=y_error;
}



void uwb_position_callback(const dji_sdk_demo::Pos::ConstPtr& msg) {



  curr_pos = *msg;
  t1 = high_resolution_clock::now();

  duration<double> time_span = duration_cast<duration<double>>(t1 - t2);
  t2 = high_resolution_clock::now();
//ROS_INFO("I heard: [%d]",( posi.nses));
   pid_pos_form(); 
   //max pitch and roll angle 0.1 rad
  //int elapsed_time = ros::Time::now().toNSec();// - posi.nses;
  //-1 pitch opposite sign
  switch(target_set_state){
    case 0:
      desired_pos.x=3.0;
      desired_pos.y=1.5;
      if((fabs(x_error)<POS_THRESH)&&(fabs(y_error)<POS_THRESH)){
          printf(" Going to state 1 (8 1,5)\n" );
          desired_pos.x=8.0;
          desired_pos.y=1.5;
          target_set_state=1;
        }
      break;
    case 1:
      desired_pos.x=8.0;
      desired_pos.y=1.5;
      if((fabs(x_error)<POS_THRESH)&&(fabs(y_error)<POS_THRESH)){
         printf(" Going to state 2 (8 3)\n" );
          target_set_state=2;
          desired_pos.x=8.0;
      	  desired_pos.y=3.0;
        }     
      break;
    case 2:
      desired_pos.x=8.0;
      desired_pos.y=3.0;
      if((fabs(x_error)<POS_THRESH)&&(fabs(y_error)<POS_THRESH)){
          printf(" Going to state 3 (3 3)\n" );
          target_set_state=3;
          desired_pos.x=3.0;
          desired_pos.y=3.0;
        }
      break;
    case 3:
      desired_pos.x=3.0;
      desired_pos.y=3.0;
      if((fabs(x_error)<POS_THRESH)&&(fabs(y_error)<POS_THRESH)){
         printf(" Going to state 1 (3.5 2.5)\n" );
          target_set_state=0;
         desired_pos.x=3.0;
     	 desired_pos.y=1.5;
        }     
      break;
    
    
  }

  
  write_data_flight();
//comm<<_span.count()<<std::endl;
  //elapsed_time=ros::Time::now().toNSec()-elapsed_time+curr_pos.time; 



}


void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  curr_imu=*msg;
  /*
    t1 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t1 - t2);
    t2 = high_resolution_clock::now();

  //  auto duration = duration_cast<microseconds>( t2 - t1 ).count()
   
  
  
  imuG<<curr_imu.linear_acceleration.x<<" "<<curr_imu.linear_acceleration.y<<" "<<curr_imu.linear_acceleration.z<<" "<<time_span.count()<<std::endl;
  //imuG<<curr_imu.angular_velocity.x<<" "<<curr_imu.angular_velocity.y<<" "<<curr_imu.angular_velocity.z<<std::endl;
  xspeed+=((curr_imu.linear_acceleration.x+ax_prev)/2.0)*time_span.count();
  yspeed+=((curr_imu.linear_acceleration.y+ay_prev)/2.0)*time_span.count();

  dximu+=((xspeed_prev+xspeed)/2.0)*time_span.count();  
  dyimu+=((yspeed_prev+yspeed)/2.0)*time_span.count(); 
  imuAcc<<dximu<<" "<<dyimu<<" "<<xspeed<<" "<<yspeed<<std::endl;
//imuAcc<<curr_imu.linear_acceleration.x<<" "<<curr_imu.linear_acceleration.y<<" "<<curr_imu.linear_acceleration.z<<" "<<dt<<std::endl;
  //imuG<<curr_imu.angular_velocity.x<<" "<<curr_imu.angular_velocity.y<<" "<<curr_imu.angular_velocity.z<<std::endl;
xspeed_prev=xspeed;
yspeed_prev=yspeed;
ax_prev=curr_imu.linear_acceleration.x;
ay_prev=curr_imu.linear_acceleration.y;
  */

}




void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}


void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
  rpy=toEulerAngle(current_atti);
  printf("Yaw is %f \n",rpy.z );
}

/*
void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  local_position=*msg;
}


void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  current_gps_position = *msg;
}

void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg) {
  current_gps_health = msg->data;
}

*/



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
