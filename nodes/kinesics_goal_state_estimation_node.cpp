// ROS includes
#include <ros/package.h>  // for finding the filepath to the package
#include <ros/ros.h>
#include <tf/transform_listener.h>

// ROS proxemics includes
#include <proxemics/KinesicsGoalState.h>

// other includes
#include <angles/angles.h>

// global variables: filepath (for models directory)
std::string g_filepath = ".";

// definitions: gesture recognition rates
#define PROBABILITY_FOLDER_GESTURE "/models/gesture"
#define BETA_NUM_GESTURE ( 7 )
#define D_NUM_GESTURE    ( 6 )

// global variables: gesture recognition rate models
float g_dist_array_gesture[ D_NUM_GESTURE ]    = { 0.4, 0.8, 1.5, 2.5, 3.5, 4.0 };
float g_beta_array_gesture[ BETA_NUM_GESTURE ] = { 0, 30, 60, 90, 120, 150, 180 };

// global variables: gesture recognition probability tables
float g_head_beta_probability[ D_NUM_GESTURE ][ BETA_NUM_GESTURE ];
float g_head_dist_probability[ D_NUM_GESTURE ];
float g_left_beta_probability[ D_NUM_GESTURE][BETA_NUM_GESTURE ];
float g_left_dist_probability[ D_NUM_GESTURE ];
float g_right_beta_probability[ D_NUM_GESTURE ][ BETA_NUM_GESTURE ];
float g_right_dist_probability[ D_NUM_GESTURE ];

// constant global variables: TF frames (note: from proxemics_controller_node)
const std::string g_robot_frame_id = "/robot/base_link";  // g_origin_frame_id in proxemics_controller_node
const std::string g_human_frame_id = "/human/base_link";  // g_target_frame_id in proxemics_controller_node

// constant global variables: interagent poses (note: from proxemics_controller_node)
double g_curr_range_robot_to_human = 0.0;  // curr_range_robot_to_human in proxemics_controller_node
double g_curr_angle_robot_to_human = 0.0;  // curr_angle_robot_to_human in proxemics_controller_node
double g_curr_angle_human_to_robot = 0.0;  // curr_angle_human_to_robot in proxemics_controller_node

// function prototypes: gesture recognition
void loadGestureRecTables();
int findIndexF( double data, float index_array[], int array_length );
void getTwoGestureIndices( double data,   float index_array[], int array_length,
                           int*   index1 = NULL, int*  index2 = NULL);
double getGestureRecRateDist( double dist, float probability[D_NUM_GESTURE]);
double getGestureRecRateBeta( double dist, double beta,
                              float probability[ D_NUM_GESTURE ][ BETA_NUM_GESTURE ]);
double getGestureRecRate( double dist, double alpha, double beta );

// function prototypes: gesture output
double calcLocOutA( double dist, double ang_ab, double ang_ba, double noise = 0.0 );
double calcLocOutB( double dist, double ang_ba, double ang_ab, double noise = 0.0 );

// function prototypes: gesture input
double calcLocInA( double dist, double ang_ab, double ang_ba, double gesture_locus );
double calcLocInB( double dist, double ang_ba, double ang_ab, double gesture_locus );


// function prototypes: interagent poses (note: from proxemics_controller_node)
double getDistance(double target_x, double target_y, double origin_x = 0.0, double origin_y = 0.0);
double getAngle(double target_x, double target_y, double origin_x = 0.0, double origin_y = 0.0, double origin_th = 0.0);
double sign(const double x);
double clip(double x, const double min_x, const double max_x);
bool   updatePoses();  // note: this function is based on the proxemics_controller_node, but is new to this program



// global variables: ROS proxemics messages
proxemics::KinesicsGoalState g_kinesics_goal_state;

// global variables: audio noise
double g_ambient_spl = 45.0;

// executes main program code
int main( int argc, char** argv )
{
  // Initialize ROS.
  ros::init( argc, argv, "proxemics_goal_state_estimation" );
  ros::NodeHandle nh;

  // Create a ROS publisher for the kinesics goal state.
  ros::Publisher pub_kinesics_goal_state  = nh.advertise<proxemics::KinesicsGoalState>(  "kinesics_goal_state",  1 );
  
  // Load speech and gesture recognition tables.
  nh.param<std::string>("/proxemics_goal_state_estimation/filepath", g_filepath, ros::package::getPath("proxemics"));
  loadGestureRecTables();
  
  // local variables: TF listener
  tf::TransformListener tf_listener;  // note: from proxemics_controller_node

  // Spin.
  ros::Rate loop_rate( 10 );
  while ( ros::ok() )
  {
	  try // get human position
    {
      if (tf_listener.canTransform(g_robot_frame_id, g_human_frame_id, ros::Time(0)))
      {
        tf::StampedTransform tf_robot_to_human;
        tf::StampedTransform tf_human_to_robot;
        tf_listener.lookupTransform(g_robot_frame_id, g_human_frame_id, ros::Time(0), tf_robot_to_human);
        tf_listener.lookupTransform(g_human_frame_id, g_robot_frame_id, ros::Time(0), tf_human_to_robot);
        
        // get range/angle to target frame
        g_curr_range_robot_to_human = getDistance(tf_robot_to_human.getOrigin().x(), tf_robot_to_human.getOrigin().y());
        g_curr_angle_robot_to_human = getAngle(tf_robot_to_human.getOrigin().x(), tf_robot_to_human.getOrigin().y());
        g_curr_angle_human_to_robot = getAngle(tf_human_to_robot.getOrigin().x(), tf_human_to_robot.getOrigin().y());
        
        //g_kinesics_goal_state.range_max = 0.5 * max_state.rng;
        //g_kinesics_goal_state.angle     = max_state.ang_ab;
        g_kinesics_goal_state.range_max = 0.5 * g_curr_range_robot_to_human;
        g_kinesics_goal_state.angle     = 0.5 * g_curr_angle_robot_to_human;
      }
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
    }
    
    pub_kinesics_goal_state.publish( g_kinesics_goal_state );

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
} // main( int, char** )



// load all the tables
void loadGestureRecTables()
{
  std::string filepath_probability_folder_gesture = g_filepath + PROBABILITY_FOLDER_GESTURE;
  std::string filepath_head_dist  = filepath_probability_folder_gesture + "/head_dist.txt";
  std::string filepath_head_beta  = filepath_probability_folder_gesture + "/head_beta.txt";
  std::string filepath_left_dist  = filepath_probability_folder_gesture + "/l_hand_dist.txt";
  std::string filepath_left_beta  = filepath_probability_folder_gesture + "/l_hand_beta.txt";
  std::string filepath_right_dist = filepath_probability_folder_gesture + "/r_hand_dist.txt";
  std::string filepath_right_beta = filepath_probability_folder_gesture + "/r_hand_beta.txt";

  // open all the files
  // note: this is VERY bad style for filenames
  FILE* head_dist  = fopen( filepath_head_dist.c_str(),  "r" );
  FILE* head_beta  = fopen( filepath_head_beta.c_str(),  "r" );
  FILE* left_dist  = fopen( filepath_left_dist.c_str(),  "r" );
  FILE* left_beta  = fopen( filepath_left_beta.c_str(),  "r" );
  FILE* right_dist = fopen( filepath_right_dist.c_str(), "r" );
  FILE* right_beta = fopen( filepath_right_beta.c_str(), "r" );

  // open all the files
  // note: this is VERY bad style for filenames
  //FILE* head_dist  = fopen( PROBABILITY_FOLDER_GESTURE "/head_dist.txt", "r" );
  //FILE* head_beta  = fopen( PROBABILITY_FOLDER_GESTURE "/head_beta.txt", "r" );
  //FILE* left_dist  = fopen( PROBABILITY_FOLDER_GESTURE "/l_hand_dist.txt", "r" );
  //FILE* left_beta  = fopen( PROBABILITY_FOLDER_GESTURE "/l_hand_beta.txt", "r" );
  //FILE* right_dist = fopen( PROBABILITY_FOLDER_GESTURE "/r_hand_dist.txt", "r" );
  //FILE* right_beta = fopen( PROBABILITY_FOLDER_GESTURE "/r_hand_beta.txt", "r" );

  // load the distance probability
  for ( int i = 0; i < D_NUM_GESTURE; ++i )
  {
    fscanf( head_dist,  "%f", &g_head_dist_probability[ i ] );
    fscanf( left_dist,  "%f", &g_left_dist_probability[ i ] );
    fscanf( right_dist, "%f", &g_right_dist_probability[ i ] );
  }

  // load the beta probability
  for ( int i = 0; i < D_NUM_GESTURE; ++i )
    for ( int j = 0; j < BETA_NUM_GESTURE; ++j )
    {
      fscanf( head_beta,  "%f", &g_head_beta_probability[ i ][ j ] );
      fscanf( left_beta,  "%f", &g_left_beta_probability[ i ][ j ] );
      fscanf( right_beta, "%f", &g_right_beta_probability[ i ][ j ] );
    }
} // loadGestureRecTables()

// find the index in the look-up table
int findIndexF( double data, float index_array[], int array_length )
{
  int i = 0;
  while ( ( data >= index_array[ i ] ) && ( i < array_length ) ) ++i;

  if ( i == 0 )
    return i;
  else if ( data >= index_array[ array_length - 1 ] )
    return array_length - 1;
  return i;
} // findIndexF(double, float, int)

// get the index for bilinear interpolation
void getTwoGestureIndices( double data,   float index_array[], int array_length,
                           int*   index1, int*  index2)
{
  if ( ( index1 == NULL ) || ( index2 == NULL ) ) return;
  int temp = findIndexF(data, index_array, array_length);
  if (temp == 0)
  {
    *index1 = 0;
    *index2 = 1;
  } else
  {
    *index1 = temp - 1;
    *index2 = temp;
  }
} // getTwoGestureIndices( double, float[], int, int*, int* )

double getGestureRecRateDist( double dist, float probability[D_NUM_GESTURE])
{
  int idx_dist1 = 0, idx_dist2 = 0;
  getTwoGestureIndices( dist, g_dist_array_gesture, D_NUM_GESTURE,
                       &idx_dist1, &idx_dist2 );

  // linear interpolation
  double a      = ( g_dist_array_gesture[ idx_dist2 ] - dist )
                / ( g_dist_array_gesture[ idx_dist2 ] - g_dist_array_gesture[ idx_dist1 ] );
  double result = probability[ idx_dist1 ] * a + probability[ idx_dist2 ] * ( 1.0 - a );
  return result;
} // getGestureRecRateDist( double, float[] )

// the probability of alpha
double getGestureRecRateAlpha( double alpha )
{
  if ( ( alpha >= 0.0 ) && ( alpha <= 20.0 ) ) return 1.0; // all tracked
  if ( alpha >= 27.0 ) return 0.0; // Nothing tracked
  double a = ( alpha - 20.0 ) / ( 27.0 - 20.0 );
  return  1.0 - a;
} // getGestureRecRateAlpha( float )

double getGestureRecRateBeta( double dist, double beta,
                              float probability[ D_NUM_GESTURE ][ BETA_NUM_GESTURE ])
{
  int idx_dist1 = 0, idx_dist2 = 0;
  int idx_beta1 = 0, idx_beta2 = 0;

  // get the index for bilinear interpolation
  getTwoGestureIndices( dist, g_dist_array_gesture, D_NUM_GESTURE,
                       &idx_dist1, &idx_dist2 );
  getTwoGestureIndices( beta, g_beta_array_gesture, BETA_NUM_GESTURE,
                       &idx_beta1, &idx_beta2 );

  // interpolation
  double a      = ( g_dist_array_gesture[ idx_dist2 ] - dist )
                / ( g_dist_array_gesture[ idx_dist2 ] - g_dist_array_gesture[ idx_dist1 ] );
  double b      = ( g_beta_array_gesture[ idx_beta2 ] - beta )
                / ( g_beta_array_gesture[ idx_beta2 ] - g_beta_array_gesture[ idx_beta1 ] );
  double result = probability[ idx_dist1 ][ idx_beta1 ] *         a   *         b
                + probability[ idx_dist1 ][ idx_beta2 ] *         a   * ( 1.0 - b )
                + probability[ idx_dist2 ][ idx_beta1 ] * ( 1.0 - a ) *         b
                + probability[ idx_dist2 ][ idx_beta2 ] * ( 1.0 - a ) * ( 1.0 - b );
  return result;
} // getGestureRecRateBeta( double, double, float[][] )

double getGestureRecRate( double dist, double alpha, double beta )
{
  if ( dist < g_dist_array_gesture[ 0 ] )
    dist = g_dist_array_gesture[ 0 ];
  if ( dist > g_dist_array_gesture[ D_NUM_GESTURE - 1 ] )
    dist = g_dist_array_gesture[ D_NUM_GESTURE - 1 ];
  alpha = angles::to_degrees( alpha );
  beta  = angles::to_degrees( beta  );

  // the probability for head
  double head_dist  = getGestureRecRateDist( dist, g_head_dist_probability );
  double head_alpha = getGestureRecRateAlpha( alpha);
  double head_beta  = getGestureRecRateBeta( dist, beta, g_head_beta_probability );
  double head_joint = head_dist * head_alpha * head_beta;

  // the probability for left hand
  double left_dist  = getGestureRecRateDist( dist, g_left_dist_probability );
  double left_alpha = getGestureRecRateAlpha( alpha );
  double left_beta  = getGestureRecRateBeta( dist, beta, g_left_beta_probability );
  double left_joint = left_dist * left_alpha * left_beta;

  // the probability for right hand
  double right_dist  = getGestureRecRateDist( dist, g_right_dist_probability );
  double right_alpha = getGestureRecRateAlpha( alpha );
  double right_beta  = getGestureRecRateBeta( dist, beta, g_right_beta_probability );
  double right_joint = right_dist * right_alpha * right_beta;

  // the joint probability for head, left hand, and right hand
  double overall_joint = head_joint * left_joint * right_joint;

  // display inputs, probabilities, and outputs
  //printf( "INPUT:\n" );
  //printf( "Dist: %4.2f, Alpha: %4.2f, Beta: %4.2f\n", dist, alpha, beta );
  //printf("\n\nOUTPUT:\n");
  //printf( "\nHead Probability:\n" );
  //printf( "Dist: %1.8f, Alpha: %1.8f, Beta: %1.8f, Joint: %1.8f\n",
  //        head_dist, head_alpha, head_beta, head_joint );
  //printf( "\nLeft Hand Probability:\n" );
  //printf( "Dist: %1.8f, Alpha: %1.8f, Beta: %1.8f, Joint: %1.8f\n",
  //        left_dist, left_alpha, left_beta, left_joint );
  //printf( "\nRight Hand Probability:\n" );
  //printf( "Dist: %1.8f, Alpha: %1.8f, Beta: %1.8f, Joint: %1.8f\n",
  //        right_dist, right_alpha, right_beta, right_joint );
  //printf( "\nOverall Probability: %1.8f\n", overall_joint );

  return overall_joint;
} // getGestureRecRate( double, double, double )

/* note: from proxemics_controller_node */

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
