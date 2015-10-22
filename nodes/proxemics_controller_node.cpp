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
std::string g_world_frame_id = "/world";

// global variables: publishers/subscribers
std::string g_pub_cmd_vel_name = "/robot/cmd_vel";

// global variables: proxemics goal state
double g_goal_range_robot_to_human = 1.5;
double g_goal_angle_robot_to_human = angles::from_degrees(0.0);
double g_goal_angle_human_to_robot = angles::from_degrees(0.0);

// global variables: min/max speeds
double g_min_speed_lin_x = 0.01;
double g_max_speed_lin_x = 1.5;
double g_min_speed_lin_y = 0.01;
double g_max_speed_lin_y = 1.5;
double g_min_speed_ang_z = angles::from_degrees(1.0);
double g_max_speed_ang_z = angles::from_degrees(180.0);

// global variables: max accelerations
double g_max_accel_lin_x = 0.1;
double g_max_accel_lin_y = 0.1;
double g_max_accel_ang_z = angles::from_degrees(10.0);

// global variables: linear/angular proportional-derivative (PD) gains
double g_gain_p_lin_x = 1.0;
double g_gain_p_lin_y = 1.0;
double g_gain_p_ang_z = 1.0;
double g_gain_d_lin_x = 0.0;
double g_gain_d_lin_y = 0.0;
double g_gain_d_ang_z = 0.0;

// global variables: TF timeout (in seconds)
double g_tf_timeout = 1.0;

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
  nh.param<std::string>("/proxemics_controller/robot_frame_id", g_robot_frame_id,            "/robot/base_link");
  nh.param<std::string>("/proxemics_controller/human_frame_id", g_human_frame_id,            "/human/base_link");
  nh.param("/proxemics_controller/tf_timeout",                  g_tf_timeout,                1.0);
  nh.param("/proxemics_controller/goal_range_robot_to_human",   g_goal_range_robot_to_human, 1.5);
  nh.param("/proxemics_controller/goal_angle_robot_to_human",   g_goal_angle_robot_to_human, 0.0);
  nh.param("/proxemics_controller/goal_angle_human_to_robot",   g_goal_angle_human_to_robot, 0.0);
  nh.param("/proxemics_controller/min_speed_lin_x",             g_min_speed_lin_x,           0.0);
  nh.param("/proxemics_controller/max_speed_lin_x",             g_max_speed_lin_x,           1.0);
  nh.param("/proxemics_controller/min_speed_lin_y",             g_min_speed_lin_y,           0.0);
  nh.param("/proxemics_controller/max_speed_lin_y",             g_max_speed_lin_y,           1.0);
  nh.param("/proxemics_controller/min_speed_ang_z",             g_min_speed_ang_z,           0.0);  // note: parameter in degrees, but variable in radians!
  nh.param("/proxemics_controller/max_speed_ang_z",             g_max_speed_ang_z,          90.0);  // note: parameter in degrees, but variable in radians!
  nh.param("/proxemics_controller/max_accel_lin_x",             g_max_accel_lin_x,           0.1);
  nh.param("/proxemics_controller/max_accel_lin_y",             g_max_accel_lin_y,           0.1);
  nh.param("/proxemics_controller/max_accel_ang_z",             g_max_accel_ang_z,          10.0);  // note: parameter in degrees, but variable in radians!
  nh.param("/proxemics_controller/gain_p_lin_x",                g_gain_p_lin_x,              1.0);
  nh.param("/proxemics_controller/gain_p_lin_y",                g_gain_p_lin_y,              1.0);
  nh.param("/proxemics_controller/gain_p_ang_z",                g_gain_p_ang_z,              1.0);
  nh.param("/proxemics_controller/gain_d_lin_x",                g_gain_d_lin_x,              0.0);
  nh.param("/proxemics_controller/gain_d_lin_y",                g_gain_d_lin_y,              0.0);
  nh.param("/proxemics_controller/gain_d_ang_z",                g_gain_d_ang_z,              0.0);
  g_min_speed_ang_z = angles::from_degrees(g_min_speed_ang_z);
  g_max_speed_ang_z = angles::from_degrees(g_max_speed_ang_z);
  g_max_accel_ang_z = angles::from_degrees(g_max_accel_ang_z);

  // initialize dynamic reconfigure parameter server
  dynamic_reconfigure::Server<proxemics::ProxemicsControllerConfig>               srv_reconfig;
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
  double curr_err_lin_x            = 0.0;
  double curr_err_lin_y            = 0.0;
  double curr_err_ang_z            = 0.0;
  double prev_err_lin_x            = curr_err_lin_x;
  double prev_err_lin_y            = curr_err_lin_y;
  double prev_err_ang_z            = curr_err_ang_z;
  double curr_vel_lin_x                 = 0.0;
  double curr_vel_lin_y                 = 0.0;
  double curr_vel_ang_z                 = 0.0;
  double prev_vel_lin_x                 = curr_vel_lin_x;
  double prev_vel_lin_y                 = curr_vel_lin_y;
  double prev_vel_ang_z                 = curr_vel_ang_z;
  double accel_lin_x                    = curr_vel_lin_x - prev_vel_lin_x;
  double accel_lin_y                    = curr_vel_lin_y - prev_vel_lin_y;
  double accel_ang_z                    = curr_vel_ang_z - prev_vel_ang_z;

  // 
  ros::Time     t_curr      = ros::Time::now();
  ros::Time     t_prev      = t_curr;
  ros::Duration dur_delta_t = t_curr - t_prev;

  while (ros::ok())
  {
    prev_vel_lin_x = curr_vel_lin_x;
    prev_vel_lin_y = curr_vel_lin_y;
    prev_vel_ang_z = curr_vel_ang_z;
    curr_vel_lin_x = curr_vel_lin_y = curr_vel_ang_z = 0.0;

    t_prev      = t_curr;
    t_curr      = ros::Time::now();
    dur_delta_t = t_curr - t_prev;
    
    try // get human position
    {
      if (tf_listener.canTransform(g_robot_frame_id, g_human_frame_id, ros::Time(0)))
      {
        tf::StampedTransform tf_world_to_human;
        tf::StampedTransform tf_world_to_robot;

        tf_listener.lookupTransform(g_world_frame_id, g_human_frame_id, ros::Time(0), tf_world_to_human);
        tf_listener.lookupTransform(g_world_frame_id, g_robot_frame_id, ros::Time(0), tf_world_to_robot);

        tf::Transform tf_robot_to_human = tf_world_to_robot.inverse() * tf_world_to_human;
        tf::Transform tf_human_to_robot = tf_world_to_human.inverse() * tf_world_to_robot;

        ros::Duration dur_robot_to_human = t_curr - tf_world_to_human.stamp_;
        ros::Duration dur_human_to_robot = t_curr - tf_world_to_robot.stamp_;

//        ros::Duration dur_robot_to_human = t_curr - tf_robot_to_human.stamp_;
//        ros::Duration dur_human_to_robot = t_curr - tf_human_to_robot.stamp_;

        if ((dur_robot_to_human.toSec() < g_tf_timeout) && (dur_human_to_robot.toSec() < g_tf_timeout))
        {

          // get range/angle to target frame
          curr_range_robot_to_human = getDistance(tf_robot_to_human.getOrigin().x(), tf_robot_to_human.getOrigin().y());
          curr_angle_robot_to_human = getAngle(tf_robot_to_human.getOrigin().x(), tf_robot_to_human.getOrigin().y());
          curr_angle_human_to_robot = getAngle(tf_human_to_robot.getOrigin().x(), tf_human_to_robot.getOrigin().y());

          ROS_INFO("(%.2f, %.2f, %.2f)",
                   curr_range_robot_to_human,
                   angles::to_degrees(curr_angle_robot_to_human),
                   angles::to_degrees(curr_angle_human_to_robot));
        
          // derivative control
          prev_err_lin_x = curr_err_lin_x;
          prev_err_lin_y = curr_err_lin_y;
          prev_err_ang_z = curr_err_ang_z;

          // proportional control: velocity(error) = gain * error
          // - note: based on control equations in Mead & Mataric (HRI 2012)
          curr_err_lin_x = cos(curr_angle_robot_to_human) * ((curr_range_robot_to_human - g_goal_range_robot_to_human) * cos(curr_angle_robot_to_human - g_goal_angle_robot_to_human))
                         - sin(curr_angle_robot_to_human) * sin(curr_angle_human_to_robot - g_goal_angle_human_to_robot);
          curr_err_lin_y = sin(curr_angle_robot_to_human) * ((curr_range_robot_to_human - g_goal_range_robot_to_human) * cos(curr_angle_robot_to_human - g_goal_angle_robot_to_human))
                         + cos(curr_angle_robot_to_human) * sin(curr_angle_human_to_robot - g_goal_angle_human_to_robot);
          curr_err_ang_z = curr_angle_robot_to_human - g_goal_angle_robot_to_human;  // note: the curr and goal angles are reversed from the paper!
        
          // scale velocities by gains
          curr_vel_lin_x = g_gain_p_lin_x * curr_err_lin_x + g_gain_d_lin_x * (curr_err_lin_x - prev_err_lin_x);
          curr_vel_lin_y = g_gain_p_lin_y * curr_err_lin_y + g_gain_d_lin_y * (curr_err_lin_y - prev_err_lin_y);
          curr_vel_ang_z = g_gain_p_ang_z * curr_err_ang_z + g_gain_d_ang_z * (curr_err_ang_z - prev_err_ang_z);
        
          // clip velocities
          clip(curr_vel_lin_x, g_min_speed_lin_x, g_max_speed_lin_x);
          clip(curr_vel_lin_y, g_min_speed_lin_y, g_max_speed_lin_y);
          clip(curr_vel_ang_z, g_min_speed_ang_z, g_max_speed_ang_z);
        }
        else
        {
          ROS_WARN("TF frames timed out...");
        }
      }
      else ROS_WARN("Lost person...");
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
    }

    // clip accelerations
    double delta_t = dur_delta_t.toSec();
    accel_lin_x    = curr_vel_lin_x - prev_vel_lin_x; 
    accel_lin_y    = curr_vel_lin_y - prev_vel_lin_y; 
    accel_ang_z    = curr_vel_ang_z - prev_vel_ang_z; 
    if (fabs(accel_lin_x) > g_max_accel_lin_x * delta_t) curr_vel_lin_x = prev_vel_lin_x + sign(accel_lin_x) * g_max_accel_lin_x * delta_t;
    if (fabs(accel_lin_y) > g_max_accel_lin_y * delta_t) curr_vel_lin_y = prev_vel_lin_y + sign(accel_lin_y) * g_max_accel_lin_y * delta_t;
    if (fabs(accel_ang_z) > g_max_accel_ang_z * delta_t) curr_vel_ang_z = prev_vel_ang_z + sign(accel_ang_z) * g_max_accel_ang_z * delta_t;

    // print proxemics goal state
    ROS_INFO("goal_state = [%.2f, %.2f, %.2f]",
             g_goal_range_robot_to_human,
             g_goal_angle_robot_to_human,
             g_goal_angle_human_to_robot);

    // print linear (vel_lin_x and vel_lin_y) and angular (vel_ang_z) velocities
    //ROS_INFO("vel = [%.2f, %.2f, %.2f]", vel_lin_x, vel_lin_y, vel_ang_z);

    // publish linear (vel_lin_x and vel_lin_y) and angular (vel_ang_z) velocities
    cmd_vel.linear.x  = curr_vel_lin_x;
    cmd_vel.linear.y  = curr_vel_lin_y;
    cmd_vel.angular.z = curr_vel_ang_z;
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

  // set TF timeout (in seconds) variable
  g_tf_timeout = config.tf_timeout;
  
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
  
  // set max acceleration variables
  g_max_accel_lin_x = config.max_accel_lin_x;
  g_max_accel_lin_y = config.max_accel_lin_y;
  g_max_accel_ang_z = angles::from_degrees(config.max_accel_ang_z);
  
  // set linear/angular proportional (P) gain variables
  g_gain_p_lin_x = config.gain_p_lin_x;
  g_gain_p_lin_y = config.gain_p_lin_y;
  g_gain_p_ang_z = config.gain_p_ang_z;
  
  // set linear/angular derivative (d) gain variables
  g_gain_d_lin_x = config.gain_d_lin_x;
  g_gain_d_lin_y = config.gain_d_lin_y;
  g_gain_d_ang_z = config.gain_d_ang_z;
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
  if (fabs(x) < min_x) return x = 0.0;  // note: this is treated more as a threshold than a lower clip!
  if (fabs(x) > max_x) return sign(x) * max_x;
  return x;
  //return ((fabs(x) < min_x) ? 0.0 : sign(x) * max_x);
} // clip(double, const double, const double)

