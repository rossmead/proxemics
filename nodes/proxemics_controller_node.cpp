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

// global variables: publishers/subscribers
std::string g_pub_cmd_vel_name = "/robot/cmd_vel";

// global variables: TF frame IDs (names)
std::string g_world_frame_id    = "/world";
std::string g_robot_frame_id    = "/robot/base_link";
std::string g_human_frame_id    = "/human/base_link";

// global variables: TF timeout (in seconds)
bool   g_enable_tf_timeout = true;
double g_tf_timeout        = 1.0;

// global variables: proxemics goal state
bool   g_enable_goal_state         = true;
double g_goal_range_robot_to_human = 1.5;
double g_goal_angle_robot_to_human = angles::from_degrees(0.0);
double g_goal_angle_human_to_robot = angles::from_degrees(0.0);

// global variables: min/max speeds
bool   g_enable_speed_limits = true;
double g_min_speed_lin_x     = 0.01;
double g_max_speed_lin_x     = 1.5;
double g_min_speed_lin_y     = 0.01;
double g_max_speed_lin_y     = 1.5;
double g_min_speed_ang_z     = angles::from_degrees(1.0);
double g_max_speed_ang_z     = angles::from_degrees(180.0);

// global variables: max accelerations
bool   g_enable_accel_limits = true;
double g_max_accel_lin_x     = 0.1;
double g_max_accel_lin_y     = 0.1;
double g_max_accel_ang_z     = angles::from_degrees(10.0);

// global variables: max jerks
bool   g_enable_jerk_limits = true;
double g_max_jerk_lin_x     = 0.1;
double g_max_jerk_lin_y     = 0.1;
double g_max_jerk_ang_z     = angles::from_degrees(10.0);

// global variables: linear/angular proportional-derivative (PD) gains
bool   g_enable_p_control = true;
bool   g_enable_d_control = true;
double g_gain_p_lin_x     = 1.0;
double g_gain_p_lin_y     = 1.0;
double g_gain_p_ang_z     = 1.0;
double g_gain_d_lin_x     = 0.0;
double g_gain_d_lin_y     = 0.0;
double g_gain_d_ang_z     = 0.0;

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
  nh.param<std::string>("/proxemics_controller/world_frame_id", g_world_frame_id,             "/world");
  nh.param<std::string>("/proxemics_controller/robot_frame_id", g_robot_frame_id,             "/robot/base_link");
  nh.param<std::string>("/proxemics_controller/human_frame_id", g_human_frame_id,             "/human/base_link");
  nh.param("/proxemics_controller/enable_tf_timeout",           g_enable_tf_timeout,          true);
  nh.param("/proxemics_controller/tf_timeout",                  g_tf_timeout,                 1.0);
  nh.param("/proxemics_controller/enable_goal_state",           g_enable_goal_state,          true);
  nh.param("/proxemics_controller/goal_range_robot_to_human",   g_goal_range_robot_to_human,  1.5);
  nh.param("/proxemics_controller/goal_angle_robot_to_human",   g_goal_angle_robot_to_human,  0.0);
  nh.param("/proxemics_controller/goal_angle_human_to_robot",   g_goal_angle_human_to_robot,  0.0);
  nh.param("/proxemics_controller/enable_speed_limits",         g_enable_speed_limits,        true);
  nh.param("/proxemics_controller/min_speed_lin_x",             g_min_speed_lin_x,            0.0);
  nh.param("/proxemics_controller/max_speed_lin_x",             g_max_speed_lin_x,            1.0);
  nh.param("/proxemics_controller/min_speed_lin_y",             g_min_speed_lin_y,            0.0);
  nh.param("/proxemics_controller/max_speed_lin_y",             g_max_speed_lin_y,            1.0);
  nh.param("/proxemics_controller/min_speed_ang_z",             g_min_speed_ang_z,            0.0);  // note: parameter in degrees, but variable in radians!
  nh.param("/proxemics_controller/max_speed_ang_z",             g_max_speed_ang_z,           90.0);  // note: parameter in degrees, but variable in radians!
  nh.param("/proxemics_controller/enable_accel_limits",         g_enable_accel_limits,       true);
  nh.param("/proxemics_controller/max_accel_lin_x",             g_max_accel_lin_x,            0.1);
  nh.param("/proxemics_controller/max_accel_lin_y",             g_max_accel_lin_y,            0.1);
  nh.param("/proxemics_controller/max_accel_ang_z",             g_max_accel_ang_z,           10.0);  // note: parameter in degrees, but variable in radians!
  nh.param("/proxemics_controller/enable_jerk_limits",          g_enable_jerk_limits,        true);
  nh.param("/proxemics_controller/max_jerk_lin_x",              g_max_jerk_lin_x,             0.1);
  nh.param("/proxemics_controller/max_jerk_lin_y",              g_max_jerk_lin_y,             0.1);
  nh.param("/proxemics_controller/max_jerk_ang_z",              g_max_jerk_ang_z,            10.0);  // note: parameter in degrees, but variable in radians!
  nh.param("/proxemics_controller/enable_p_control",            g_enable_p_control,          true);
  nh.param("/proxemics_controller/gain_p_lin_x",                g_gain_p_lin_x,               1.0);
  nh.param("/proxemics_controller/gain_p_lin_y",                g_gain_p_lin_y,               1.0);
  nh.param("/proxemics_controller/gain_p_ang_z",                g_gain_p_ang_z,               1.0);
  nh.param("/proxemics_controller/enable_d_control",            g_enable_d_control,          true);
  nh.param("/proxemics_controller/gain_d_lin_x",                g_gain_d_lin_x,               0.0);
  nh.param("/proxemics_controller/gain_d_lin_y",                g_gain_d_lin_y,               0.0);
  nh.param("/proxemics_controller/gain_d_ang_z",                g_gain_d_ang_z,               0.0);
  g_min_speed_ang_z = angles::from_degrees(g_min_speed_ang_z);
  g_max_speed_ang_z = angles::from_degrees(g_max_speed_ang_z);
  g_max_accel_ang_z = angles::from_degrees(g_max_accel_ang_z);
  g_max_jerk_ang_z  = angles::from_degrees(g_max_jerk_ang_z);

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
  double                loop_hz = 100;
  ros::Rate             loop_rate(loop_hz);

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
  double curr_vel_lin_x            = 0.0;
  double curr_vel_lin_y            = 0.0;
  double curr_vel_ang_z            = 0.0;
  double prev_vel_lin_x            = curr_vel_lin_x;
  double prev_vel_lin_y            = curr_vel_lin_y;
  double prev_vel_ang_z            = curr_vel_ang_z;
  double curr_accel_lin_x          = curr_vel_lin_x - prev_vel_lin_x;
  double curr_accel_lin_y          = curr_vel_lin_y - prev_vel_lin_y;
  double curr_accel_ang_z          = curr_vel_ang_z - prev_vel_ang_z;
  double prev_accel_lin_x          = curr_accel_lin_x;
  double prev_accel_lin_y          = curr_accel_lin_y;
  double prev_accel_ang_z          = curr_accel_ang_z;
  double jerk_lin_x                = curr_vel_lin_x - prev_vel_lin_x;
  double jerk_lin_y                = curr_vel_lin_y - prev_vel_lin_y;
  double jerk_ang_z                = curr_vel_ang_z - prev_vel_ang_z;

  //  
  ros::Time     t_curr      = ros::Time::now();
  ros::Time     t_prev      = t_curr;
  ros::Duration dur_delta_t = t_curr - t_prev;
  double        delta_t     = dur_delta_t.toSec();

  while (ros::ok())
  {
    prev_vel_lin_x = curr_vel_lin_x;
    prev_vel_lin_y = curr_vel_lin_y;
    prev_vel_ang_z = curr_vel_ang_z;
    curr_vel_lin_x = curr_vel_lin_y = curr_vel_ang_z = 0.0;

    t_prev      = t_curr;
    t_curr      = ros::Time::now();
    dur_delta_t = t_curr - t_prev;
    delta_t     = dur_delta_t.toSec();
    
    if (delta_t < 0.000001) delta_t = 1.0 / loop_hz;
    //ROS_INFO("delta_t = %.8f", delta_t);
    
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

        if ((!g_enable_tf_timeout) || ((dur_robot_to_human.toSec() < g_tf_timeout) && (dur_human_to_robot.toSec() < g_tf_timeout)))
        {

          // get range/angle to target frame
          curr_range_robot_to_human = getDistance(tf_robot_to_human.getOrigin().x(), tf_robot_to_human.getOrigin().y());
          curr_angle_robot_to_human = getAngle(   tf_robot_to_human.getOrigin().x(), tf_robot_to_human.getOrigin().y());
          curr_angle_human_to_robot = getAngle(   tf_human_to_robot.getOrigin().x(), tf_human_to_robot.getOrigin().y());

          //ROS_INFO("(%.2f, %.2f, %.2f)",
          //         curr_range_robot_to_human,
          //         angles::to_degrees(curr_angle_robot_to_human),
          //         angles::to_degrees(curr_angle_human_to_robot));
        
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
        
          // add proportional (P) gains to the velocities
          if (g_enable_p_control)
          {
            curr_vel_lin_x += (g_gain_p_lin_x * curr_err_lin_x);
            curr_vel_lin_y += (g_gain_p_lin_y * curr_err_lin_y);
            curr_vel_ang_z += (g_gain_p_ang_z * curr_err_ang_z);
	      }
        
          // add derivative (D) gains to the velocities
          if (g_enable_d_control)
          {
            curr_vel_lin_x += (g_gain_d_lin_x * (curr_err_lin_x - prev_err_lin_x) / delta_t);
            curr_vel_lin_y += (g_gain_d_lin_y * (curr_err_lin_y - prev_err_lin_y) / delta_t);
            curr_vel_ang_z += (g_gain_d_ang_z * (curr_err_ang_z - prev_err_ang_z) / delta_t);
	      }
        
          // clip velocities
          if (g_enable_speed_limits)
          {
            curr_vel_lin_x = clip(curr_vel_lin_x, g_min_speed_lin_x, g_max_speed_lin_x);
            curr_vel_lin_y = clip(curr_vel_lin_y, g_min_speed_lin_y, g_max_speed_lin_y);
            curr_vel_ang_z = clip(curr_vel_ang_z, g_min_speed_ang_z, g_max_speed_ang_z);
	      }
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
    prev_accel_lin_x = curr_accel_lin_x;
    prev_accel_lin_y = curr_accel_lin_y;
    prev_accel_ang_z = curr_accel_ang_z;
    curr_accel_lin_x = (curr_vel_lin_x - prev_vel_lin_x) / delta_t;
    curr_accel_lin_y = (curr_vel_lin_y - prev_vel_lin_y) / delta_t;
    curr_accel_ang_z = (curr_vel_ang_z - prev_vel_ang_z) / delta_t;
    if (g_enable_accel_limits)
    {
      if (fabs(curr_accel_lin_x) > g_max_accel_lin_x)
      {
		curr_accel_lin_x = sign(curr_accel_lin_x) * g_max_accel_lin_x;
	  }
      if (fabs(curr_accel_lin_y) > g_max_accel_lin_y)
      {
		curr_accel_lin_y = sign(curr_accel_lin_y) * g_max_accel_lin_y;
	  }
      if (fabs(curr_accel_ang_z) > g_max_accel_ang_z)
      {
		curr_accel_ang_z = sign(curr_accel_ang_z) * g_max_accel_ang_z;
	  }
    }

    // clip jerks
    jerk_lin_x = (curr_accel_lin_x - prev_accel_lin_x) / delta_t;
    jerk_lin_y = (curr_accel_lin_y - prev_accel_lin_y) / delta_t;
    jerk_ang_z = (curr_accel_ang_z - prev_accel_ang_z) / delta_t;
    if (g_enable_jerk_limits)
    {
      if (fabs(jerk_lin_x) > g_max_jerk_lin_x)
      {
        jerk_lin_x       = sign(jerk_lin_x) * g_max_jerk_lin_x;
        curr_accel_lin_x = prev_accel_lin_x + jerk_lin_x * delta_t;
      }
      if (fabs(jerk_lin_y) > g_max_jerk_lin_y)
      {
        jerk_lin_y       = sign(jerk_lin_y) * g_max_jerk_lin_y;
        curr_accel_lin_y = prev_accel_lin_y + jerk_lin_y * delta_t;
      }
      if (fabs(jerk_ang_z) > g_max_jerk_ang_z)
      {
        jerk_ang_z       = sign(jerk_ang_z) * g_max_jerk_ang_z;
        curr_accel_ang_z = prev_accel_ang_z + jerk_ang_z * delta_t;
      }
    }

    // clip velocities
    curr_vel_lin_x = prev_vel_lin_x + curr_accel_lin_x * delta_t;
    curr_vel_lin_y = prev_vel_lin_y + curr_accel_lin_y * delta_t;
    curr_vel_ang_z = prev_vel_ang_z + curr_accel_ang_z * delta_t;
    if (g_enable_speed_limits)
    {
      curr_vel_lin_x = clip(curr_vel_lin_x, g_min_speed_lin_x, g_max_speed_lin_x);
      curr_vel_lin_y = clip(curr_vel_lin_y, g_min_speed_lin_y, g_max_speed_lin_y);
      curr_vel_ang_z = clip(curr_vel_ang_z, g_min_speed_ang_z, g_max_speed_ang_z);
    }

    // print proxemics goal state
    //ROS_INFO("goal_state = [%.2f, %.2f, %.2f]",
    //         g_goal_range_robot_to_human,
    //         g_goal_angle_robot_to_human,
    //         g_goal_angle_human_to_robot);

    // print linear (x and y) and angular (z) velocities
    ROS_INFO("vel = [%.2f, %.2f, %.2f]",
             curr_vel_lin_x, curr_vel_lin_y, curr_vel_ang_z);

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
  g_world_frame_id    = config.world_frame_id;
  g_robot_frame_id    = config.robot_frame_id;
  g_human_frame_id    = config.human_frame_id;

  // set TF timeout (in seconds) variable
  g_enable_tf_timeout = config.tf_timeout;
  g_tf_timeout        = config.tf_timeout;
  
  // set proxemics goal state variables
  g_enable_goal_state         = config.enable_goal_state;
  if (g_enable_goal_state)
  {
    g_goal_range_robot_to_human = config.goal_range_robot_to_human;
    g_goal_angle_robot_to_human = angles::from_degrees(config.goal_angle_robot_to_human);
    g_goal_angle_human_to_robot = angles::from_degrees(config.goal_angle_human_to_robot);
  }
  
  // set min/max speed variables
  g_enable_speed_limits = config.enable_speed_limits;
  if (g_enable_speed_limits)
  {
    g_min_speed_lin_x = config.min_speed_lin_x;
    g_max_speed_lin_x = config.max_speed_lin_x;
    g_min_speed_lin_y = config.min_speed_lin_y;
    g_max_speed_lin_y = config.max_speed_lin_y;
    g_min_speed_ang_z = angles::from_degrees(config.min_speed_ang_z);
    g_max_speed_ang_z = angles::from_degrees(config.max_speed_ang_z);
  }
  
  // set max acceleration variables
  g_enable_accel_limits = config.enable_accel_limits;
  if (g_enable_accel_limits)
  {
    g_max_accel_lin_x = config.max_accel_lin_x;
    g_max_accel_lin_y = config.max_accel_lin_y;
    g_max_accel_ang_z = angles::from_degrees(config.max_accel_ang_z);
  }
  
  // set max jerk variables
  g_enable_jerk_limits = config.enable_jerk_limits;
  if (g_enable_jerk_limits)
  {
    g_max_jerk_lin_x = config.max_jerk_lin_x;
    g_max_jerk_lin_y = config.max_jerk_lin_y;
    g_max_jerk_ang_z = angles::from_degrees(config.max_jerk_ang_z);
  }
  
  // set linear/angular proportional (P) gain variables
  g_enable_p_control = config.enable_p_control;
  if (g_enable_p_control)
  {
    g_gain_p_lin_x = config.gain_p_lin_x;
    g_gain_p_lin_y = config.gain_p_lin_y;
    g_gain_p_ang_z = config.gain_p_ang_z;
  }
  
  // set linear/angular derivative (d) gain variables
  g_enable_d_control = config.enable_d_control;
  if (g_enable_d_control)
  {
    g_gain_d_lin_x = config.gain_d_lin_x;
    g_gain_d_lin_y = config.gain_d_lin_y;
    g_gain_d_ang_z = config.gain_d_ang_z;
  }
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
  if (!std::isfinite(x)) return 0.0;
  if (fabs(x) < min_x)   return 0.0;  // note: this is treated more as a threshold than a lower clip!
  if (fabs(x) > max_x)   return sign(x) * max_x;
  return x;
} // clip(double, const double, const double)

