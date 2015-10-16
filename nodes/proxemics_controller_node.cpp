// standard includes
#include <iostream>

// ROS includes
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

// ROS dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <proxemics/ProxemicsControllerConfig.h>

// ROS proxemics includes
#include <proxemics/ProxemicsGoalState.h>

// namespaces
using namespace std;

// global variables: TF frame IDs (names)
std::string g_robot_frame_id = "/robot/base_link";
std::string g_human_frame_id = "/human/base_link";

// global variables: publishers/subscribers
std::string g_pub_cmd_vel_name = "/robot/cmd_vel";

// global variables: proxemics goal state
double g_goal_range_robot_to_human = 1.5;
double g_goal_angle_robot_to_human = 0.0 * M_PI / 180.0;
double g_goal_angle_human_to_robot = 0.0 * M_PI / 180.0;

// global variables: min/max speeds
double g_min_speed_lin_x = 0.01;
double g_max_speed_lin_x = 1.5;
double g_min_speed_lin_y = 0.01;
double g_max_speed_lin_y = 1.5;
double g_min_speed_ang_z = angles::from_degrees(1.0);
double g_max_speed_ang_z = angles::from_degrees(180.0);

// global variables: linear/angular gains
double g_gain_p_lin_x   = 1.0;
double g_gain_p_lin_y   = 1.0;
double g_gain_p_ang_z   = 1.0;

// callback function prototypes
void cbProxemicsGoalState(const proxemics::ProxemicsGoalState::ConstPtr &proxemics_goal_state);
void cbReconfigure(proxemics::ProxemicsControllerConfig &config, uint32_t level);

// function prototypes
double getDistance(double target_x, double target_y, double origin_x = 0.0, double origin_y = 0.0);
double getAngle(double target_x, double target_y, double origin_x = 0.0, double origin_y = 0.0, double origin_th = 0.0);
double sign(const double x);
double clip(double x, const double min_x, const double max_x);

// executes main program code
int main(int argc, char** argv)
{

  // initialize ROS
  ros::init(argc, argv, "proxemics_controller");
  ros::NodeHandle nh;

  // initialize parameters
  nh.setParam("goal_range_robot_to_human", g_goal_range_robot_to_human);
  nh.setParam("goal_angle_robot_to_human", g_goal_angle_robot_to_human);
  nh.setParam("goal_angle_human_to_robot", g_goal_angle_human_to_robot);
  nh.setParam("min_speed_lin_x", g_min_speed_lin_x);
  nh.setParam("max_speed_lin_x", g_max_speed_lin_x);
  nh.setParam("min_speed_lin_y", g_min_speed_lin_y);
  nh.setParam("max_speed_lin_y", g_max_speed_lin_y);
  nh.setParam("min_speed_ang_z", g_min_speed_ang_z);
  nh.setParam("max_speed_ang_z", g_max_speed_ang_z);
  nh.setParam("gain_p_lin_x", g_gain_p_lin_x);
  nh.setParam("gain_p_lin_y", g_gain_p_lin_y);
  nh.setParam("gain_p_ang_z", g_gain_p_ang_z);

  // initialize dynamic reconfigure parameter server
  dynamic_reconfigure::Server<proxemics::ProxemicsControllerConfig> srv_reconfig;
  dynamic_reconfigure::Server<proxemics::ProxemicsControllerConfig>::CallbackType cb_reconfig;
  cb_reconfig = boost::bind(&cbReconfigure, _1, _2);
  srv_reconfig.setCallback(cb_reconfig);

  // initialize publishers
  ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>(g_pub_cmd_vel_name, 1);
  geometry_msgs::Twist cmd_vel;
  
  // initialize subscribers
  ros::Subscriber sub = nh.subscribe("proxemics_goal_state", 1, cbProxemicsGoalState);

  // 
  tf::TransformListener tf_listener;
  ros::Rate loop_rate(100);

  //
  double curr_range_robot_to_human = 0.0;
  double curr_angle_robot_to_human = 0.0;
  double curr_angle_human_to_robot = 0.0;
  double vel_lin_x = 0.0;
  double vel_lin_y = 0.0;
  double vel_ang_z = 0.0;

  while (ros::ok())
  {
    vel_lin_x = vel_lin_y = vel_ang_z = 0.0;

    try // get human position
    {
      if (tf_listener.canTransform(g_robot_frame_id, g_human_frame_id, ros::Time(0)))
      {
        tf::StampedTransform tf_robot_to_human;
        tf::StampedTransform tf_human_to_robot;
        tf_listener.lookupTransform(g_robot_frame_id, g_human_frame_id, ros::Time(0), tf_robot_to_human);
        tf_listener.lookupTransform(g_human_frame_id, g_robot_frame_id, ros::Time(0), tf_human_to_robot);

        // get range/angle to target frame
        curr_range_robot_to_human = getDistance(tf_robot_to_human.getOrigin().x(), tf_robot_to_human.getOrigin().y());
        curr_angle_robot_to_human = getAngle(tf_robot_to_human.getOrigin().x(), tf_robot_to_human.getOrigin().y());
        curr_angle_human_to_robot = getAngle(tf_human_to_robot.getOrigin().x(), tf_human_to_robot.getOrigin().y());

        // proportional control: velocity(error) = gain * error
        // - note: based on control equations in Mead & Mataric (HRI 2012)
        vel_lin_x  = cos(curr_angle_robot_to_human) * ((curr_range_robot_to_human - g_goal_range_robot_to_human) * cos(curr_angle_robot_to_human - g_goal_angle_robot_to_human))
                   - sin(curr_angle_robot_to_human) * sin(curr_angle_human_to_robot - g_goal_angle_human_to_robot);
        vel_lin_y  = sin(curr_angle_robot_to_human) * ((curr_range_robot_to_human - g_goal_range_robot_to_human) * cos(curr_angle_robot_to_human - g_goal_angle_robot_to_human))
                   + cos(curr_angle_robot_to_human) * sin(curr_angle_human_to_robot - g_goal_angle_human_to_robot);
        vel_ang_z  = curr_angle_robot_to_human - g_goal_angle_robot_to_human;  // note: the curr and goal angles are reversed from the paper!
        
        // scale velocities by gains
        vel_lin_x *= g_gain_p_lin_x;
        vel_lin_y *= g_gain_p_lin_y;
        vel_ang_z *= g_gain_p_ang_z;
        
        // clip velocities
        clip(vel_lin_x, g_min_speed_lin_x, g_max_speed_lin_x);
        clip(vel_lin_y, g_min_speed_lin_y, g_max_speed_lin_y);
        clip(vel_ang_z, g_min_speed_ang_z, g_max_speed_ang_z);
      }
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
    }
    
    // print proxemics goal state
    ROS_INFO("goal_state = [%.2f, %.2f, %.2f]",
             g_goal_range_robot_to_human,
             g_goal_angle_robot_to_human,
             g_goal_angle_human_to_robot);

    // print linear (vel_lin_x and vel_lin_y) and angular (vel_ang_z) velocities
    //ROS_INFO("vel = [%.2f, %.2f, %.2f]", vel_lin_x, vel_lin_y, vel_ang_z);

    // publish linear (vel_lin_x and vel_lin_y) and angular (vel_ang_z) velocities
    cmd_vel.linear.x  = vel_lin_x;
    cmd_vel.linear.y  = vel_lin_y;
    cmd_vel.angular.z = vel_ang_z;
    pub_cmd_vel.publish(cmd_vel);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
} // main(int, char**)

void cbProxemicsGoalState(const proxemics::ProxemicsGoalState::ConstPtr &proxemics_goal_state)
{
  g_goal_range_robot_to_human = proxemics_goal_state->range_robot_to_human;
  g_goal_angle_robot_to_human = proxemics_goal_state->angle_robot_to_human;
  g_goal_angle_human_to_robot = proxemics_goal_state->angle_human_to_robot;
} // cbProxemicsGoalState(const proxemics::ProxemicsGoalState::ConstPtr &)

void cbReconfigure(proxemics::ProxemicsControllerConfig &config, uint32_t level)
{
  // set TF frame ID (name) variables
  g_robot_frame_id = config.robot_frame_id;
  g_human_frame_id = config.human_frame_id;
  
  // set proxemics goal state variables
  g_goal_range_robot_to_human = config.goal_range_robot_to_human;
  g_goal_angle_robot_to_human = angles::from_degrees(config.goal_angle_robot_to_human);
  g_goal_angle_human_to_robot = angles::from_degrees(config.goal_angle_human_to_robot);
  
  // set min/max speed variables
  g_min_speed_lin_x = config.min_speed_lin_x;
  g_max_speed_lin_x = config.max_speed_lin_x;
  g_min_speed_lin_y = config.min_speed_lin_y;
  g_max_speed_lin_y = config.max_speed_lin_y;
  g_min_speed_ang_z = angles::from_degrees(config.min_speed_ang_z);
  g_max_speed_ang_z = angles::from_degrees(config.max_speed_ang_z);
  
  // set linear/angular gain variables
  g_gain_p_lin_x = config.gain_p_lin_x;
  g_gain_p_lin_y = config.gain_p_lin_y;
  g_gain_p_ang_z = config.gain_p_ang_z;
} // cbReconfigure(proxemics::ProxemicsControllerConfig &, uint32_t)

double getDistance(double target_x, double target_y, double origin_x, double origin_y)
{
  return sqrt(pow(target_x - origin_x, 2) + pow(target_y - origin_y, 2));
} // getDistance(double, double, double, double)

double getAngle(double target_x, double target_y, double origin_x, double origin_y, double origin_th)
{
  return angles::normalize_angle(atan2(target_y - origin_y, target_x - origin_x) - origin_th);
} // getAngle(double, double, double, double, double)

double sign(double x)
{
  return ((x < 0.0) ? -1.0 : 1.0);
} // sign(double)

double clip(double x, const double min_x, const double max_x)
{
  return ((fabs(x) < min_x) ? 0.0 : sign(x) * max_x);
} // clip(double, const double, const double)
