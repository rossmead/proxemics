// standard includes
#include <iostream>

// ROS includes
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

// ROS proxemics includes
#include <proxemics/ProxemicsGoalState.h>

// namespaces
using namespace std;

// constant global variables (TF frames)
const std::string g_origin_frame_id = "/robot/base_link";
const std::string g_target_frame_id = "/human/base_link";

// constant global variables (publishers/subscribers)
const std::string g_pub_cmd_vel_name = "/robot/cmd_vel";

// global variables (proxemics goal state)
double g_goal_range_robot_to_human = 1.5;
double g_goal_angle_robot_to_human = 0.0 * M_PI / 180.0;
double g_goal_angle_human_to_robot = 0.0 * M_PI / 180.0;

// constant global variables (min/max speeds)
const double g_min_speed_lin_x = 0.01;
const double g_min_speed_lin_y = 0.01;
const double g_min_speed_ang_z = 1.0 * M_PI / 180.0;
const double g_max_speed_lin_x = 1.5;
const double g_max_speed_lin_y = 1.5;
const double g_max_speed_ang_z = 1.0 * M_PI;

// callback function prototypes
void cbProxemicsGoalState(const proxemics::ProxemicsGoalState::ConstPtr &proxemics_goal_state);

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
  bool   is_holonomic = true;
  double gain_lin_x   = 1.0;
  double gain_lin_y   = 1.0;
  double gain_ang_z   = 1.0;

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
      if (tf_listener.canTransform(g_origin_frame_id, g_target_frame_id, ros::Time(0)))
      {
        tf::StampedTransform tf_robot_to_human;
        tf::StampedTransform tf_human_to_robot;
        tf_listener.lookupTransform(g_origin_frame_id, g_target_frame_id, ros::Time(0), tf_robot_to_human);
        tf_listener.lookupTransform(g_target_frame_id, g_origin_frame_id, ros::Time(0), tf_human_to_robot);

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
        vel_lin_x *= gain_lin_x;
        vel_lin_y *= gain_lin_y;
        vel_ang_z *= gain_ang_z;
        
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
