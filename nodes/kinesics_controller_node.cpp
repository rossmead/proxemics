// NOTE: This controller supports the "kinesic_controller" (without the "s" after "kinesic")
//       developed by Edward Kaszubski for the Bandit robot platform in the USC Interaction Lab.
//       This implementation is limited solely to this framework -- it is not general!

// standard includes
#include <fcntl.h>
#include <unistd.h>

// ROS includes
#include <angles/angles.h>
#include <ros/ros.h>

// ROS dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <proxemics/KinesicsControllerConfig.h>

// ROS proxemics includes
#include <proxemics/KinesicsGoalState.h>



// global variables
bool   g_set_vh_centers     = true;
double g_range_min          =   0.3;
double g_range_max          =   0.5;
double g_range              =   0.5 * (g_range_min + g_range_max);
double g_v_center_range_min =  45.0;
double g_v_center_range_max =  80.0;
double g_h_center_range_min =   5.0;
double g_h_center_range_max =  -5.0;
double g_v_center           = g_v_center_range_max;
double g_h_center           = g_v_center_range_max;



// callback function prototypes
void cbKinesicsGoalState(const proxemics::KinesicsGoalState::ConstPtr &kinesics_goal_state);
void cbReconfigure(proxemics::KinesicsControllerConfig &config, uint32_t level);



// function prototypes
double clip(double x, const double x_min, const double x_max);



int main(int argc, char** argv)
{

  // initialize ROS
  ros::init(argc, argv, "kinesics_controller");
  ros::NodeHandle nh;
  
  // initialize subscribers
  ros::Subscriber sub = nh.subscribe("kinesics_goal_state", 1, cbKinesicsGoalState);

  // initialize dynamic reconfigure parameter server
  dynamic_reconfigure::Server<proxemics::KinesicsControllerConfig>               srv_reconfig;
  dynamic_reconfigure::Server<proxemics::KinesicsControllerConfig>::CallbackType cb_reconfig;
  cb_reconfig = boost::bind(&cbReconfigure, _1, _2);
  srv_reconfig.setCallback(cb_reconfig);
  
  // initialize parameters
  nh.param("/kinesics_controller/range_min",                            g_range_min,           0.3);
  nh.param("/kinesics_controller/range_max",                            g_range_max,           0.5);
  nh.param("/kinesics_controller/v_center_range_min",                   g_v_center_range_min, 45.0);
  nh.param("/kinesics_controller/v_center_range_max",                   g_v_center_range_max, 80.0);
  nh.param("/kinesics_controller/h_center_range_min",                   g_h_center_range_min,  5.0);
  nh.param("/kinesics_controller/h_center_range_max",                   g_h_center_range_max, -5.0);
  nh.param("/kinesic_controller/reconfigure/arm_beat_gesture_v_center", g_v_center,           g_v_center_range_max);
  nh.param("/kinesic_controller/reconfigure/arm_beat_gesture_h_center", g_h_center,           g_h_center_range_max);
  
  //
  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
	  if (g_set_vh_centers)
	  {
      g_v_center = g_v_center_range_min 
                                  + (g_v_center_range_max - g_v_center_range_min) 
                                  * (g_range - g_range_min) / (g_range_max - g_range_min);
      
      g_h_center = g_h_center_range_min 
                                  + (g_h_center_range_max - g_h_center_range_min) 
                                  * pow((g_range - g_range_min) / (g_range_max - g_range_min), 2);
      
      nh.setParam("/kinesic_controller/reconfigure/arm_beat_gesture_v_center", g_v_center);
      nh.setParam("/kinesic_controller/reconfigure/arm_beat_gesture_h_center", g_h_center);
      
	  	g_set_vh_centers = false;
      
      ROS_INFO("(v, h) center = (%.1lf, %.1lf)",
               g_v_center, g_h_center);
	  }
	  ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
} // main(int, char**)



void cbKinesicsGoalState(const proxemics::KinesicsGoalState::ConstPtr &kinesics_goal_state)
{
  g_range          = clip(kinesics_goal_state->range, g_range_min, g_range_max);
  g_set_vh_centers = true;
} // cbKinesicsGoalState(const proxemics::KinesicsGoalState::ConstPtr &)



void cbReconfigure(proxemics::KinesicsControllerConfig &config, uint32_t level)
{
  // set min/max range variables
  g_range_min = config.range_min;
  g_range_max = config.range_max;
  g_range     = clip(g_range, g_range_min, g_range_max);
  
  // set arm beat gesture vertical (v) center at min/max range variables
  g_v_center_range_min = config.v_center_range_min;
  g_v_center_range_max = config.v_center_range_max;
  
  // set arm beat gesture horizontal (h) center at min/max range variables
  g_h_center_range_min = config.h_center_range_min;
  g_h_center_range_max = config.h_center_range_max;
  
  g_set_vh_centers = true;
} // cbReconfigure(proxemics::KinesicsControllerConfig &, uint32_t)



double clip(double x, const double x_min, const double x_max)
{
  if (x < x_min) x = x_min;
  if (x > x_max) x = x_max;
  return x;
} // clip(double, const double, const double)
