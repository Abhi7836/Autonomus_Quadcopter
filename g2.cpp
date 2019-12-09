
#include <cstdlib>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>



sensor_msgs::NavSatFix global_pose;
void global_pose_cb(const sensor_msgs::NavSatFixConstPtr& msg)
{
  global_pose = *msg;
  ROS_INFO("got global position [%d]: %f, %f, %f", global_pose.header.seq, global_pose.latitude, global_pose.longitude, global_pose.altitude);
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
  ROS_INFO("got status: %d, %d", (int)msg->armed, (int)msg->connected);
}


int main(int argc, char **argv)
{

  int rate_hz = 10;

  ros::init(argc, argv, "global_pos_mission");
  ros::NodeHandle n;
  ros::NodeHandle& nh = n;
  ros::Rate rate(rate_hz);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                              ("mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                 ("mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                     ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                       ("mavros/set_mode", 1);
  ros::Subscriber global_pose_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, global_pose_cb);

  global_pose.header.seq = 0;

  // wait for FCU connection
  while(ros::ok() && (!current_state.connected || global_pose.header.seq == 0))
  {
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("got global position [%d]: %f, %f, %f", global_pose.header.seq, global_pose.latitude, global_pose.longitude, global_pose.altitude);

  ////////////////////////////////////////////
  ///////////////////ARM//////////////////////
  ////////////////////////////////////////////
  ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_msgs::CommandBool srv;
  srv.request.value = true;
  if(arming_cl.call(srv))
  {
    ROS_INFO("ARM send ok %d", srv.response.success);
  }
  else
  {
    ROS_ERROR("Failed arming or disarming");
  }


  ////////////////////////////////////////////
  /////////////////CLEAR MISSION/////////////////
  ////////////////////////////////////////////


  ros::ServiceClient wp_clear_client = n.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");
  mavros_msgs::WaypointClear wp_clear_srv;
  if (wp_clear_client.call(wp_clear_srv))
  {
    ROS_INFO("Waypoint list was cleared");
  }
  else
  {
    ROS_ERROR("Waypoint list couldn't been cleared");
  }

  ////////////////////////////////////////////
  /////////////////DO MISSION/////////////////
  ////////////////////////////////////////////
  ros::ServiceClient client_wp = n.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");

  mavros_msgs::WaypointPush srv_wp;
  srv_wp.request.start_index = 0;
  mavros_msgs::CommandHome set_home_srv;

  mavros_msgs::Waypoint wp;
  // fill wp
  wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
  wp.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
  wp.is_current   = true;
  wp.autocontinue = true;
  wp.param1       = 5;
  wp.z_alt        = 6;
  wp.x_lat        = global_pose.latitude;
  wp.y_long       = global_pose.longitude;
  srv_wp.request.waypoints.push_back(wp);
  wp.command      = mavros_msgs::CommandCode::NAV_WAYPOINT;
  wp.is_current   = false;
  wp.autocontinue = true;
  wp.param1       = 5;          // hold time sec
  wp.z_alt        = 6;
  wp.x_lat        = 25.2598966;
  wp.y_long       = 82.9888723;
  srv_wp.request.waypoints.push_back(wp);
  wp.x_lat        = 25.2599929;
  wp.y_long       = 82.9889875;
  srv_wp.request.waypoints.push_back(wp);
  wp.x_lat        = 25.2599072;
  wp.y_long       = 82.9891143;
  srv_wp.request.waypoints.push_back(wp);
  wp.x_lat        = 25.2598637;
  wp.y_long       = 82.9890239;
  srv_wp.request.waypoints.push_back(wp);
  wp.command      = mavros_msgs::CommandCode::NAV_LAND;
  wp.z_alt        = 0;
  wp.x_lat        = 25.2598637;
  wp.y_long       = 82.9890239;
  srv_wp.request.waypoints.push_back(wp);

  if (client_wp.call(srv_wp))
  {
    // ok, check srv.response
    ROS_INFO("Uploaded WPs!");
    mavros_msgs::State current_state;
  }
  else
  {
    // roscpp error
    ROS_ERROR("Upload Failed");
  }


  ////////////////////////////////////////////
  ////////////////////////////////////////////
  ///////////SET MODE:AUTO.MISSION////////////
  ////////////////////////////////////////////
  ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  mavros_msgs::SetMode srv_setMode;
  srv_setMode.request.base_mode = 0; //(uint8_t)mavros_msgs::SetModeRequest::MAV_MODE_AUTO_ARMED;
  srv_setMode.request.custom_mode = "AUTO.MISSION";
  if(cl.call(srv_setMode))
  {
    ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);
  }
  else
  {
    ROS_ERROR("Failed SetMode");
    return -1;
  }


  ////////////////////////////////////////////
  ////////////////////////////////////////////
  ///////////WAIT UNTIL DISARMED//////////////
  ////////////////////////////////////////////
  current_state.armed = true;
  while(ros::ok() && current_state.armed )
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Drone disarmed");


  return 0;
}


