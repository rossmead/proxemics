// standard includes
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// STL includes
#include <vector>

// GSL includes
#include <gsl/gsl_cdf.h>
#include <gsl/gsl_randist.h>

// ROS includes
#include <ros/package.h>  // for finding the filepath to the package
#include <ros/ros.h>
#include <tf/transform_listener.h>

// ROS proxemics includes
#include <proxemics/ProxemicsGoalState.h>
#include <proxemics/VocalicsGoalState.h>
#include <proxemics/KinesicsGoalState.h>

// PCL includes
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>

// other includes
#include <angles/angles.h>

// definitions: speaker input
#define MODEL_L ( "/models/orientation_left.txt" )
#define MODEL_R ( "/models/orientation_right.txt" )
#define HRTF_NUM   ( 13 )
#define DIRECT_NUM ( 13 )

// global variables: filepath (for models directory)
std::string g_filepath = ".";

// global variables: speaker input models
float g_model_data_l[ DIRECT_NUM ][ HRTF_NUM ];
float g_model_data_r[ DIRECT_NUM ][ HRTF_NUM ];
int   g_hrtf[ HRTF_NUM ]          = { 0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180 };
int   g_directivity[ DIRECT_NUM ] = { 0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180 };

// definitions: speech recognition rates
#define DATA_DIR_PERF        ( "/models/speech" )
#define HRTF_NUM_PERF        ( 7 )
#define DIRECTIVITY_NUM_PERF ( 7 )
#define PRESSURE_NUM_PERF    ( 7 )
#define NOISE_NUM_PERF       ( 7 )

// global variables: speech recognition rate models
int g_hrtf_perf[ HRTF_NUM_PERF ]               = {  0, 30, 60, 90, 120, 150, 180 };
int g_directivity_perf[ DIRECTIVITY_NUM_PERF ] = {  0, 30, 60, 90, 120, 150, 180 };
int g_pressure_level_perf[ PRESSURE_NUM_PERF ] = { 42, 48, 54, 60,  66,  72,  78 };
int g_noise_level_perf[ NOISE_NUM_PERF ]       = { 42, 48, 54, 60,  66,  72,  78 };

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

// data structures: speech recognition performance
struct TPerformance
{
  float performance[ PRESSURE_NUM_PERF ][ NOISE_NUM_PERF ];
};// TPerformance

// global variables: speech recognition performance
TPerformance g_performance_tables_l[ DIRECTIVITY_NUM_PERF ][ HRTF_NUM_PERF ];
TPerformance g_performance_tables_r[ DIRECTIVITY_NUM_PERF ][ HRTF_NUM_PERF ];

// constant global variables: TF frames (note: from proxemics_controller_node)
const std::string g_robot_frame_id = "/robot/base_link";  // g_origin_frame_id in proxemics_controller_node
const std::string g_human_frame_id = "/human/base_link";  // g_target_frame_id in proxemics_controller_node

// constant global variables: interagent poses (note: from proxemics_controller_node)
double g_curr_range_robot_to_human = 0.0;  // curr_range_robot_to_human in proxemics_controller_node
double g_curr_angle_robot_to_human = 0.0;  // curr_angle_robot_to_human in proxemics_controller_node
double g_curr_angle_human_to_robot = 0.0;  // curr_angle_human_to_robot in proxemics_controller_node

// function prototypes: speech recognition
void loadSpeechRecTables();
void getTwoSpeechIndeces( float data, int index_array[], int array_length,
                          int*  index1 = NULL,           int* index2 = NULL );
double bilinearInterpolation( double d11, double d12, double d21, double d22,
                              double a,   double b);
float getSpeechRecRate( TPerformance table[ DIRECTIVITY_NUM_PERF ][ HRTF_NUM_PERF ],
                        double       distance,
                        double       speaker_orientation,
                        double       listener_orientation,
                        double       speech_pressure,
                        double       noise );

// load all the table
void loadGestureRecTables();
int findIndexF( double data, float index_array[], int array_length );
void getTwoGestureIndices( double data,   float index_array[], int array_length,
                           int*   index1 = NULL, int*  index2 = NULL);
double getGestureRecRateDist( double dist, float probability[D_NUM_GESTURE]);
double getGestureRecRateBeta( double dist, double beta,
                              float probability[ D_NUM_GESTURE ][ BETA_NUM_GESTURE ]);
double getGestureRecRate( double dist, double alpha, double beta );

// function prototypes: callbacks
void cbCloud( const sensor_msgs::PointCloud2ConstPtr & input );

// function prototypes: gesture output
double calcLocOutA( double dist, double ang_ab, double ang_ba, double noise = 0.0 );
double calcLocOutB( double dist, double ang_ba, double ang_ab, double noise = 0.0 );

// function prototypes: gesture input
double calcLocInA( double dist, double ang_ab, double ang_ba, double gesture_locus );
double calcLocInB( double dist, double ang_ba, double ang_ab, double gesture_locus );

// function prototypes: speaker output
double calcSplOutA( double dist, double ang_ab, double ang_ba, double noise = 0.0 );
double calcSplOutB( double dist, double ang_ba, double ang_ab, double noise = 0.0 );

// function prototypes: speaker input
#define EAR_LEFT  (  1 )
#define EAR_RIGHT ( -1 )
#define EAR_MAX   (  0 )
void loadModel();
int findIndex( double data, int index_array[], int array_length );
double getDistanceEffect( double dist );
double getOrientationEffect( double speaker_orientation,
                             double listener_orientation,
                             float model[ DIRECT_NUM ][ HRTF_NUM ] );
double calcSplInA( double distance,
                   double speaker_orientation,
                   double listener_orientation,
                   double speech_pressure,
                   int    return_code = EAR_MAX );
double calcSplInB( double distance,
                   double speaker_orientation,
                   double listener_orientation,
                   double speech_pressure,
                   int    return_code = EAR_MAX );


// function prototypes: interagent poses (note: from proxemics_controller_node)
double getDistance(double target_x, double target_y, double origin_x = 0.0, double origin_y = 0.0);
double getAngle(double target_x, double target_y, double origin_x = 0.0, double origin_y = 0.0, double origin_th = 0.0);
double sign(const double x);
double clip(double x, const double min_x, const double max_x);
bool   updatePoses();  // note: this function is based on the proxemics_controller_node, but is new to this program


// data structures: proxemic state
namespace proxemics
{
  const double spl_noise            = 45.0;
  const double loc_noise            =  0.0;
  const double rng_mean_hh          =  1.65;
  const double rng_std_dev_hh       =  0.225;
  const double rng_mean_hr          =  1.9438;
  const double rng_std_dev_hr       =  0.6114;
  const double rng_mean             = rng_mean_hh;
  const double rng_std_dev          = rng_std_dev_hh;
  const double ang_ab_mean          = angles::from_degrees(  0.0 );
  const double ang_ab_std_dev       = angles::from_degrees( 15.0 );
  const double ang_ba_mean          = ang_ab_mean;
  const double ang_ba_std_dev       = ang_ab_std_dev;
  const double spl_out_b_mean_hh    = 65.80;
  const double spl_out_b_std_dev_hh = 6.68;
  const double spl_out_b_mean_hr    = 67.23;
  const double spl_out_b_std_dev_hr = 5.90;
  const double spl_out_a_mean       = spl_out_b_mean_hh;
  const double spl_out_a_std_dev    = spl_out_b_std_dev_hh;
  const double spl_out_b_mean       = spl_out_b_mean_hr;
  const double spl_out_b_std_dev    = spl_out_b_std_dev_hr;
  const double spl_in_a_mean        = calcSplInA( rng_mean, ang_ba_mean, ang_ab_mean, spl_out_b_mean );
  const double spl_in_a_std_dev     = fabs( calcSplInA( rng_mean, ang_ba_mean, ang_ab_mean, spl_out_b_mean + spl_out_b_std_dev ) - spl_in_a_mean );
  const double spl_in_b_mean        = calcSplInB( rng_mean, ang_ab_mean, ang_ba_mean, spl_out_b_mean );
  const double spl_in_b_std_dev     = fabs( calcSplInA( rng_mean, ang_ab_mean, ang_ba_mean, spl_out_a_mean + spl_out_a_std_dev ) - spl_in_b_mean );
  const double loc_out_a_mean       = calcLocOutA( rng_mean, ang_ab_mean, ang_ba_mean, loc_noise );
  const double loc_out_a_std_dev    = angles::from_degrees( 15.0 );
  const double loc_out_b_mean       = calcLocOutB( rng_mean, ang_ba_mean, ang_ab_mean, loc_noise );
  const double loc_out_b_std_dev    = angles::from_degrees( 15.0 );
  const double loc_in_a_mean        = calcLocInA( rng_mean, ang_ba_mean, ang_ab_mean, loc_out_b_mean );
  const double loc_in_a_std_dev     = angles::from_degrees( 15.0 );
  const double loc_in_b_mean        = calcLocInB( rng_mean, ang_ab_mean, ang_ba_mean, loc_out_a_mean );
  const double loc_in_b_std_dev     = angles::from_degrees( 15.0 );

  struct ProxemicState
  {
    double rng;        // range (meters) from A to B
    double ang_ab;     // orientation (radians) from A to B
    double ang_ba;     // orientation (radians) from B to A
    double spl_out_a;  // sound pressure level (SPL) output from A to B
    double spl_out_b;  // sound pressure level (SPL) output from B to A
    double spl_in_a;   // sound pressure level (SPL) input from B to A
    double spl_in_b;   // sound pressure level (SPL) input from A to B
    double loc_out_a;  // gesture locus (multivariate PDF) output from A to B
    double loc_out_b;  // gesture locus (multivariate PDF) output from B to A
    double loc_in_a;   // gesture locus (multivariate PDF) input from B to A
    double loc_in_b;   // gesture locus (multivariate PDF) input from A to B
    double spl_noise;  // auditory noise (SPL) from the environment
    double loc_noise;  // visual noise (gesture locus) from the environment

    void addNoise()
    {
      rng       += gsl_cdf_gaussian_Pinv( rand() / double( RAND_MAX ), rng_std_dev );
      ang_ab    += gsl_cdf_gaussian_Pinv( rand() / double( RAND_MAX ), ang_ab_std_dev );
      ang_ba    += gsl_cdf_gaussian_Pinv( rand() / double( RAND_MAX ), ang_ba_std_dev );
      spl_out_a += gsl_cdf_gaussian_Pinv( rand() / double( RAND_MAX ), spl_out_a_std_dev );
      spl_out_b += gsl_cdf_gaussian_Pinv( rand() / double( RAND_MAX ), spl_out_b_std_dev );
      spl_in_a  += gsl_cdf_gaussian_Pinv( rand() / double( RAND_MAX ), spl_in_a_std_dev );
      spl_in_b  += gsl_cdf_gaussian_Pinv( rand() / double( RAND_MAX ), spl_in_b_std_dev );
      loc_out_a += gsl_cdf_gaussian_Pinv( rand() / double( RAND_MAX ), loc_out_a_std_dev );
      loc_out_b += gsl_cdf_gaussian_Pinv( rand() / double( RAND_MAX ), loc_out_b_std_dev );
      loc_in_a  += gsl_cdf_gaussian_Pinv( rand() / double( RAND_MAX ), loc_in_a_std_dev );
      loc_in_b  += gsl_cdf_gaussian_Pinv( rand() / double( RAND_MAX ), loc_in_b_std_dev );
    } // addNoise()

    long double eval()
    {
      long double p_rng       = evalRng();
      long double p_ang_ab    = evalAngAB();
      long double p_ang_ba    = evalAngBA();
      long double p_spl_out_a = evalSplOutA();
      long double p_spl_out_b = evalSplOutB();
      long double p_spl_in_a  = evalSplInA();
      long double p_spl_in_b  = evalSplInB();
      long double p_loc_out_a = evalLocOutA();
      long double p_loc_out_b = evalLocOutB();
      long double p_loc_in_a  = evalLocInA();
      long double p_loc_in_b  = evalLocInB();
      long double p           = p_spl_out_a
                              * p_spl_in_a
                              * p_spl_out_b
                              * p_spl_in_b
                              * p_loc_out_a
                              * p_loc_in_a
                              * p_loc_out_b
                              * p_loc_in_b;
      return      p;// = p_rng * p_ang_ab * p_ang_ba;
    } // eval()

    long double eval( ProxemicState state )
    {
      long double p_rng       = evalRng( state.rng );
      long double p_ang_ab    = evalAngAB( state.ang_ab );
      long double p_ang_ba    = evalAngBA( state.ang_ba );
      long double p_spl_out_a = evalSplOutA( state.spl_out_a );
      long double p_spl_out_b = evalSplOutB( state.spl_out_b );
      long double p_spl_in_a  = evalSplInA( state.spl_in_a );
      long double p_spl_in_b  = evalSplInB( state.spl_in_b );
      long double p_loc_out_a = evalLocOutA( state.loc_out_a );
      long double p_loc_out_b = evalLocOutB( state.loc_out_b );
      long double p_loc_in_a  = evalLocInA( state.loc_in_a );
      long double p_loc_in_b  = evalLocInB( state.loc_in_b );
      long double p           = p_rng
                              * p_ang_ab
                              * p_ang_ba
                              * p_spl_out_a
                              * p_spl_in_a
                              * p_spl_out_b
                              * p_spl_in_b
                              * p_loc_out_a
                              * p_loc_in_a
                              * p_loc_out_b
                              * p_loc_in_b;
      return      p * eval();
    } // eval( ProxemicState )

    long double evalLog()
    {
      long double log_p_rng       = log( evalRng() );
      long double log_p_ang_ab    = log( evalAngAB() );
      long double log_p_ang_ba    = log( evalAngBA() );
      long double log_p_spl_out_a = log( evalSplOutA() );
      long double log_p_spl_out_b = log( evalSplOutB() );
      long double log_p_spl_in_a  = log( evalSplInA() );
      long double log_p_spl_in_b  = log( evalSplInB() );
      long double log_p_loc_out_a = log( evalLocOutA() );
      long double log_p_loc_out_b = log( evalLocOutB() );
      long double log_p_loc_in_a  = log( evalLocInA() );
      long double log_p_loc_in_b  = log( evalLocInB() );
      long double log_p           = log_p_spl_out_a
                                  + log_p_spl_in_a
                                  + log_p_spl_out_b
                                  + log_p_spl_in_b
                                  + log_p_loc_out_a
                                  + log_p_loc_in_a
                                  + log_p_loc_out_b
                                  + log_p_loc_in_b;
      return      log_p;// = log_p_rng + log_p_ang_ab + log_p_ang_ba;
    } // evalLog()

    long double evalLog( ProxemicState state )
    {
      long double log_p_rng       = log( evalRng(   state.rng    ) );
      long double log_p_ang_ab    = log( evalAngAB( state.ang_ab ) );
      long double log_p_ang_ba    = log( evalAngBA( state.ang_ba ) );
      long double log_p_spl_out_a = log( evalSplOutA( state.spl_out_a ) );
      long double log_p_spl_out_b = log( evalSplOutB( state.spl_out_b ) );
      long double log_p_spl_in_a  = log( evalSplInA( state.spl_in_a ) );
      long double log_p_spl_in_b  = log( evalSplInB( state.spl_in_b ) );
      long double log_p_loc_out_a = log( evalLocOutA( state.loc_out_a ) );
      long double log_p_loc_out_b = log( evalLocOutB( state.loc_out_b ) );
      long double log_p_loc_in_a  = log( evalLocInA( state.loc_in_a ) );
      long double log_p_loc_in_b  = log( evalLocInB( state.loc_in_b ) );
      long double log_p           = log_p_rng
                                  + log_p_ang_ab
                                  + log_p_ang_ba
                                  + log_p_spl_out_a
                                  + log_p_spl_in_a
                                  + log_p_spl_out_b
                                  + log_p_spl_in_b
                                  + log_p_loc_out_a
                                  + log_p_loc_in_a
                                  + log_p_loc_out_b
                                  + log_p_loc_in_b;
      return      log_p + evalLog();
    } // evalLog( ProxemicState )

    double evalRng( double mean = proxemics::rng_mean )
    {
      return gsl_ran_gaussian_pdf( rng - mean, rng_std_dev );
    } // evalRng()

    double evalAngAB( double mean = proxemics::ang_ab_mean )
    {
      return gsl_ran_gaussian_pdf( fabs( ang_ab ) - fabs( mean ), ang_ab_std_dev );  // bimodal? Von Mises?
    } // evalAngAB

    double evalAngBA( double mean = proxemics::ang_ba_mean )
    {
      return gsl_ran_gaussian_pdf( fabs( ang_ba ) - fabs( mean ), ang_ba_std_dev );  // bimodal? Von Mises?
    } // evalAngBA()

    double evalSplOutA( double mean = proxemics::spl_out_a_mean )
    {
      return gsl_ran_gaussian_pdf( spl_out_a - mean, spl_out_a_std_dev );
    } // evalSplOutA()

    double evalSplInA( double mean = proxemics::spl_in_a_mean )
    {
      // when should noise be factored in??? absolute value of angles???
      return getSpeechRecRate( g_performance_tables_l, rng, fabs( ang_ba ), fabs( ang_ab ), spl_out_b, spl_noise );
      double spl_l = calcSplInA( rng, ang_ba, ang_ab, spl_out_b, EAR_LEFT  );
      double spl_r = calcSplInA( rng, ang_ba, ang_ab, spl_out_b, EAR_RIGHT );
      //return gsl_ran_gaussian_pdf( spl_in_a - mean, spl_in_a_std_dev );
      return gsl_ran_gaussian_pdf( spl_l - mean, spl_in_a_std_dev ) *
             gsl_ran_gaussian_pdf( spl_r - mean, spl_in_a_std_dev );
    } // evalSplInA()

    double evalSplOutB( double mean = proxemics::spl_out_b_mean )
    {
      return gsl_ran_gaussian_pdf( spl_out_b - mean, spl_out_b_std_dev );
    } // evalSplOutB()

    double evalSplInB( double mean = proxemics::spl_in_a_mean )
    {
      double spl_l = calcSplInB( rng, ang_ab, ang_ba, spl_out_a, EAR_LEFT  );
      double spl_r = calcSplInB( rng, ang_ab, ang_ba, spl_out_a, EAR_RIGHT );
      //return gsl_ran_gaussian_pdf( spl_in_b - mean, spl_in_b_std_dev );
      return gsl_ran_gaussian_pdf( spl_l - mean, spl_in_b_std_dev ) *
             gsl_ran_gaussian_pdf( spl_r - mean, spl_in_b_std_dev );
    } // evalSplInB()

    double evalLocOutA( double mean = proxemics::loc_out_a_mean )
    {
      return gsl_ran_gaussian_pdf( fabs( loc_out_a ) - fabs( mean ), loc_out_a_std_dev );  // bimodal? Von Mises?
    } // evalLocOutA()

    double evalLocInA( double mean = proxemics::loc_in_a_mean )
    {
      return getGestureRecRate( rng, fabs( ang_ab ), fabs( ang_ba ) );

      double rng_x = rng * cos( ang_ab );
      if ( ( rng_x < 0.5 ) || ( rng_x > 4.0 ) ) return 0.0;

      int    n_joints   = 3;
      double p_ba       = cos( 0.5 * ang_ba );
      double p_max      = 0.9 * p_ba;
      double p_mid      = 0.5 * p_ba;
      double p_min      = 0.0 * p_ba;
      double abs_ang_ab = fabs( angles::to_degrees( ang_ab ) );

      if ( abs_ang_ab > 20.0 )
      {
        if ( abs_ang_ab <= 23.0 )
        {
          double t = ( 23.0 - abs_ang_ab ) / ( 23.0 - 20.0 );
          return pow( t * p_max + ( 1.0 - t ) * p_mid, n_joints );
        }

        if ( abs_ang_ab <= 27.0 )
        {
          double t = ( 27.0 - abs_ang_ab ) / ( 27.0 - 23.0 );
          return pow( t * p_mid + ( 1.0 - t ) * p_min, n_joints );
        }

        return 0.0;
      }

      return pow( p_max, n_joints );
    } // evalLocInA()

    double evalLocOutB( double mean = proxemics::loc_out_b_mean )
    {
      return gsl_ran_gaussian_pdf( fabs( loc_out_b ) - fabs( mean ), loc_out_b_std_dev );  // bimodal? Von Mises?
    } // evalLocOutB()

    double evalLocInB( double mean = proxemics::loc_in_b_mean )
    {
      if ( fabs( ang_ba ) > angles::from_degrees( 90.0 ) ) return 0.0;
      return 1.0;
      return cos( ang_ba ) * gsl_ran_gaussian_pdf( fabs( loc_in_b ) - fabs( mean ), loc_in_b_std_dev );  // bimodal? Von Mises?
    } // evalLocInB()
  };// ProxemicState
};// proxemics

// function prototypes: particles
double evalParticle( pcl::PointXYZ particle );
proxemics::ProxemicState getParticleAsProxemicState( pcl::PointXYZ particle );
pcl::PointCloud<pcl::PointXYZ> updateParticles( pcl::PointCloud<pcl::PointXYZ> pc_prev,
                                                pcl::PointXYZ                 &particle );
pcl::PointXYZ addProxemicNoiseToParticle( pcl::PointXYZ particle );

// global variables: ROS publishers
ros::Publisher g_pub_cloud;

// global variables: ROS proxemics messages
proxemics::ProxemicsGoalState g_proxemics_goal_state;
proxemics::VocalicsGoalState g_vocalics_goal_state;
proxemics::KinesicsGoalState g_kinesics_goal_state;

// executes main program code
int main( int argc, char** argv )
{
  /*
  printf( "robot[spl_out] = %.4f, %.4f\n", proxemics::spl_out_a_mean, proxemics::spl_out_a_std_dev );
  printf( "human[spl_out] = %.4f, %.4f\n", proxemics::spl_out_b_mean, proxemics::spl_out_b_std_dev );
  printf( "robot[spl_in]  = %.4f, %.4f\n", proxemics::spl_in_a_mean,  proxemics::spl_in_a_std_dev );
  printf( "human[spl_in]  = %.4f, %.4f\n", proxemics::spl_in_b_mean,  proxemics::spl_in_b_std_dev);
  return 0;
  */

  // Initialize ROS.
  ros::init( argc, argv, "proxemics_goal_state_estimation" );
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud.
  ros::Subscriber sub_cloud = nh.subscribe ( "input", 1, cbCloud );

  // Create a ROS publisher for the output point cloud.
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2>( "output", 1 );
  ros::Publisher pub_particles = nh.advertise<sensor_msgs::PointCloud2>( "particles", 1 );

  // Create a ROS publisher for the proxemics, vocalics, and kinesics goal states.
  ros::Publisher pub_proxemics_goal_state = nh.advertise<proxemics::ProxemicsGoalState>( "proxemics_goal_state", 1 );
  ros::Publisher pub_vocalics_goal_state  = nh.advertise<proxemics::VocalicsGoalState>(  "vocalics_goal_state",  1 );
  ros::Publisher pub_kinesics_goal_state  = nh.advertise<proxemics::KinesicsGoalState>(  "kinesics_goal_state",  1 );

  // load speech and gesture recognition tables
  nh.param<std::string>("filepath", g_filepath, ros::package::getPath("proxemics"));
  loadSpeechRecTables();
  loadGestureRecTables();

  // Create a point cloud.
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.points.clear();

  // PDF values.
  float x_mean    = 1.0f;
  float x_std_dev = 1.0f;
  float y_mean    = 0.0f;
  float y_std_dev = 1.5f;

  // Generate the data.
  float width  = 10.0f;
  float height = 10.0f;
  float x_min  = -0.5f * width,  x_max =  0.5f * width,  x_int =  0.1f;
  float y_min  = -0.5f * height, y_max =  0.5f * height, y_int =  0.1f;
  float z_min  =  0.0f,          z_max =  1.0f;
  float p_max  =  0.0f;
  for ( float x = x_min; x <= x_max; x += x_int )
  {
    for ( float y = y_min; y <= y_max; y += y_int)
    {
      pcl::PointXYZ point;
      point.x = x;
      point.y = y;

      float log_p_x = log( gsl_ran_gaussian_pdf( point.x - x_mean, x_std_dev ) );
      float log_p_y = log( gsl_ran_gaussian_pdf( point.y - y_mean, y_std_dev ) );

      float p_curr = evalParticle( point );
      if (p_curr > p_max) p_max = p_curr;
      point.z = p_curr * pow( 10.0, 5 );

      pcl_cloud.points.push_back( point );
    }
  }
  pcl_cloud.width  = uint32_t( pcl_cloud.points.size() );
  pcl_cloud.height = 1;

  ROS_INFO( "p_max = %.8f\n", p_max );

  pcl::PointCloud<pcl::PointXYZ> pcl_particles( pcl_cloud );
  pcl::PointXYZ                  pcl_particle;
  
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
      }
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
    }
  
    pcl_particles = updateParticles( pcl_particles, pcl_particle );

    sensor_msgs::PointCloud2 ros_cloud;
    //***//sensor_msgs::convertPointCloudToPointCloud2( pcl_cloud, ros_cloud );
    pcl::toROSMsg( pcl_cloud, ros_cloud );

    sensor_msgs::PointCloud2 ros_particles;
    //***//sensor_msgs::convertPointCloudToPointCloud2( pcl_particles, ros_particles );
    pcl::toROSMsg( pcl_particles, ros_particles );

    ros_cloud.header.frame_id     = "human/base_link";
    ros_particles.header.frame_id = "human/base_link";

    g_pub_cloud.publish( ros_cloud );
    pub_particles.publish( ros_particles );
    
    pub_proxemics_goal_state.publish( g_proxemics_goal_state );
    pub_vocalics_goal_state.publish( g_vocalics_goal_state );
    pub_kinesics_goal_state.publish( g_kinesics_goal_state );

    loop_rate.sleep();
    ros::spinOnce();
  }

  /*

  srand(time(NULL));

  unsigned int n = 10;
  double sigma = 1.0;
  double x = 0.0;
  std::vector<double> samples(n, 0.0);
  double mean = 0.0;
  double std_dev = 0.0;

  for (unsigned int i = 0; i < n; ++i)
  {
    x = (rand()) / double(RAND_MAX);
    samples[i] = gsl_cdf_gaussian_Pinv(x, sigma);
    mean += samples[i];
    printf("x = %.8f, samples[%d] = %.8f, p(sample) = %.8f, cdf(sample) = %.8f\n",
           x, i, samples[i], gsl_ran_gaussian_pdf(samples[i], sigma), gsl_cdf_gaussian_P(samples[i], sigma));
  }
  mean /= double(n);

  for (unsigned int i = 0; i < n; ++i) std_dev += pow(samples[i] - mean, 2);
  std_dev /= double(n);
  std_dev = sqrt(std_dev);

  printf("mean = %.8f, std_dev = %.8f\n", mean, std_dev);

  */

  return 0;
} // main( int, char** )

// callback function: point cloud
void cbCloud( const sensor_msgs::PointCloud2ConstPtr & ros_cloud_in )
{ // ROSS: You changed the pcl::PointCloud/fromROSMsg to sensor_msgs::... Search for "toROSMsg" as well...
  // See documentation here: http://wiki.ros.org/pcl/Overview
  //sensor_msgs::PointCloud pcl_cloud_in, pcl_cloud_out;
  //sensor_msgs::convertPointCloud2ToPointCloud( *ros_cloud_in, pcl_cloud_in );
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_in, pcl_cloud_out;
  pcl::fromROSMsg( *ros_cloud_in, pcl_cloud_in );

  // ... do data processing ...

  //sensor_msgs::PointCloud2 ros_cloud_out;
  //pcl::toROSMsg( pcl_cloud_out, ros_cloud_out );
  //g_pub_cloud.publish( ros_cloud_out );  // publish the data
} // cbCloud( const sensor_msgs::PointCloud2ConstPtr & )

double evalParticle( pcl::PointXYZ particle )
{
  return getParticleAsProxemicState( particle ).eval();
} // evalParticle( pcl::PointXYZ )

proxemics::ProxemicState getParticleAsProxemicState( pcl::PointXYZ particle )
{
  proxemics::ProxemicState state;
  state.rng       = sqrt( pow( particle.x, 2 ) + pow( particle.y, 2 ) );  // assumes xy w.r.t. person!
  state.ang_ab    = proxemics::ang_ab_mean;
  state.ang_ba    = atan2( particle.y, particle.x );  // assumes xy w.r.t. person!
  state.spl_out_a = calcSplOutA( state.rng, state.ang_ab, state.ang_ba, proxemics::spl_noise );
  state.spl_out_b = calcSplOutB( state.rng, state.ang_ba, state.ang_ab, proxemics::spl_noise );
  state.spl_in_a  = calcSplInA( state.rng, state.ang_ba, state.ang_ab, state.spl_out_a );
  state.spl_in_b  = calcSplInB( state.rng, state.ang_ab, state.ang_ba, state.spl_out_b );
  state.loc_out_a = calcLocOutA( state.rng, state.ang_ab, state.ang_ba, proxemics::loc_noise );
  state.loc_out_b = calcLocOutA( state.rng, state.ang_ba, state.ang_ab, proxemics::loc_noise );
  state.loc_in_a  = calcLocInA( state.rng, state.ang_ba, state.ang_ab, state.loc_out_a );
  state.loc_in_b  = calcLocInA( state.rng, state.ang_ab, state.ang_ba, state.loc_out_b );
  state.spl_noise = proxemics::spl_noise;
  state.loc_noise = proxemics::loc_noise;
  return state;
} // getParticleAsProxemicState( pcl::PointXYZ )

pcl::PointCloud<pcl::PointXYZ> updateParticles( pcl::PointCloud<pcl::PointXYZ> pc_prev,
                                                pcl::PointXYZ                 &particle )
{
  static bool first_time = true;

  pcl::PointCloud<pcl::PointXYZ> pc_temp;
  pcl::PointCloud<pcl::PointXYZ> pc_curr;
  long double                    total_weight   = 0.0l;
  long double                    max_log_weight = -DBL_MAX;
  pcl::PointXYZ                  max_particle;
  proxemics::ProxemicState       max_state;
  for ( uint32_t i = 0, n = pc_prev.points.size(); i < n; ++i )
  {
    pcl::PointXYZ            p     = addProxemicNoiseToParticle( pc_prev.points[ i ] );
    proxemics::ProxemicState state = getParticleAsProxemicState( p );
    //state.addNoise();

    long double log_weight = 0.0l;
    if ( !first_time )
      log_weight  = state.evalLog( getParticleAsProxemicState( particle ) );
    else
      log_weight  = state.evalLog();
    total_weight += exp( log_weight ) * pow( 10.0, 9 );
    p.z           = exp( log_weight ) * pow( 10.0, 9 );
    pc_temp.points.push_back( p );

    if ( log_weight > max_log_weight )
    {
      max_log_weight = log_weight;
      max_particle   = p;
      max_state      = state;
    }
  }
/*
  max_log_weight -= log( total_weight );
  max_particle.z /= total_weight;
  for ( uint32_t i = 0, n = pc_temp.points.size(); i < n; ++i )
    pc_temp.points[ i ].z /= total_weight;
*/
  // resample
  for ( uint32_t i = 0, n = pc_temp.points.size(); i < n; ++i )
  {
    long double rand_val = total_weight * double( rand() ) / double( RAND_MAX );
    long double curr_weight = 0.0l;
    for ( uint32_t j = 0; j < n; ++j )
    {
      curr_weight += pc_temp.points[ j ].z;
      if ( rand_val < curr_weight )
      {
        pc_curr.points.push_back( pc_temp.points[ j ] );
        break;
      }
    }
  }

  first_time = false;
  particle   = max_particle;
  //printf( "particle = %.2f, %.2f, %.2f\n", particle.x, particle.y, particle.z );

  printf( "---------------------------------------------------------------\n" );
  printf( "rng = %.2f, ang_ab = %.0f, ang_ba = %.0f\n",
          max_state.rng,
          angles::to_degrees( max_state.ang_ab ),
          angles::to_degrees( max_state.ang_ba ) );
  printf( "spl_out_a = %.2f, spl_in_a = %.2f, loc_out_a = %.0f, loc_in_a = %.0f\n",
          max_state.spl_out_a,
          max_state.spl_in_a,
          angles::to_degrees( max_state.loc_out_a ),
          angles::to_degrees( max_state.loc_in_a ) );
  printf( "spl_out_b = %.2f, spl_in_b = %.2f, loc_out_b = %.0f, loc_in_b = %.0f\n",
          max_state.spl_out_b,
          max_state.spl_in_b,
          angles::to_degrees( max_state.loc_out_b ),
          angles::to_degrees( max_state.loc_in_b ) );
  printf( "weight = %0.20f\n", exp( max_log_weight ) );
  
  g_proxemics_goal_state.range_robot_to_human = max_state.rng;
  g_proxemics_goal_state.angle_robot_to_human = max_state.ang_ab;
  g_proxemics_goal_state.angle_human_to_robot = max_state.ang_ba;
  
  //g_vocalics_goal_state.sound_pressure_level = max_state.spl_out_a;
  g_vocalics_goal_state.sound_pressure_level = calcSplOutA( 
    g_curr_range_robot_to_human, 
    g_curr_angle_robot_to_human, 
    g_curr_angle_human_to_robot,
    proxemics::spl_noise );
  
  //g_kinesics_goal_state.range_max = 0.5 * max_state.rng;
  //g_kinesics_goal_state.angle     = max_state.ang_ab;
  g_kinesics_goal_state.range_max = 0.5 * g_curr_range_robot_to_human;
  g_kinesics_goal_state.angle     = 0.5 * g_curr_angle_robot_to_human;

  return pc_curr;
} // updateParticles( pcl::PointCloud<pcl::PointXYZ )

pcl::PointXYZ addProxemicNoiseToParticle( pcl::PointXYZ particle )
{
  double rand_rng = gsl_cdf_gaussian_Pinv( rand() / double( RAND_MAX ), proxemics::rng_std_dev );
  double rand_ang = gsl_cdf_gaussian_Pinv( rand() / double( RAND_MAX ), proxemics::ang_ba_std_dev );
  particle.x     += rand_rng * cos( rand_ang );
  particle.y     += rand_rng * sin( rand_ang );
  return particle;
} // addProxemicNoiseToParticle( pcl::PointXYZ )

double calcLocOutA( double dist, double ang_ab, double ang_ba, double noise )
{
  return ang_ab;
} // calcLocOutA( double, double, double, double )

double calcLocOutB( double dist, double ang_ba, double ang_ab, double noise )
{
  return ang_ba;
} // calcLocOutA( double, double, double, double )

double calcLocInA( double dist, double ang_ba, double ang_ab, double gesture_locus )
{
  return ang_ba;
} // calcLocInA( double, double, double, double )

double calcLocInB( double dist, double ang_ab, double ang_ba, double gesture_locus )
{
  return ang_ab;
} // calcLocInB( double, double, double, double )

double calcSplOutA( double dist, double ang_ab, double ang_ba, double noise )
{
  return calcSplOutB( dist, ang_ba, ang_ab, noise );
} // calcSplOutA( double, double, double, double)

/*
 *
 *  The parameters for this function are:
 *      distance noise
 *
 *  [1] is used to estimate speech-noise ratio;
 *  [2] is used to estimate speech-distance ratio;
 *
 *  Reference:
 *  [1] Influence of Sound Immersion and Communicative Interaction on the Lombard Effect
 *  [2] Acoustic Effects of Variation in vocal effort by men, women, and children
 */
double calcSplOutB( double dist, double ang_ba, double ang_ab, double noise )
{
  //const double init_spl         = 50.0;                    // SPL when the the distance is 1m and the noise can be neglected
  //const double std_dev          =  5.0;                    // Standard deviation. Assume to be constant
  //const double dist_close       =  1.5;                    // Distance smaller than this will have less speech level increase.
  //const double dist_slope_close =  1.3 / log2( 5.0 );      // Speech level increase when distance is smaller than dist_close per 1m
  //const double dist_slope       = 26.9 / log2( 125.0 );    // Speech level increase when distance is larger than dist_close per 1m
  const double noise_slope      = 15.2 / ( 70.0 - 45.0 );  // Speech level increase per 1db noise

  const double init_spl_model      = proxemics::spl_out_b_mean;
  const double std_dev_model       = proxemics::spl_out_b_std_dev;
  const double dist_slope_model_hh = ( 66.37 - 63.63 ) / ( 5.0 - 0.5 );
  const double dist_slope_model_hr = ( 65.78 - 75.20 ) / ( 5.0 - 0.5 );
  const double dist_slope_model    = dist_slope_model_hh;

  double dist_effect = 0.0;
  //if (dist  <  0.3) dist  = 0.3;  // 0.3m is the minimum distance allowed
  //if ( dist <= dist_close )
  //  dist_effect += log2( dist / 1.0 ) * dist_slope_close;
  //else
  //  dist_effect += log2( 1.5 / 1.0 ) * dist_slope + log2( dist / 1.5 ) * dist_slope;

  //dist       += gsl_cdf_gaussian_Pinv( double( rand() ) / double( RAND_MAX ), proxemics::rng_std_dev );
  dist_effect = ( dist - proxemics::rng_mean ) * dist_slope_model;
  //printf( "Distance Effect: %+4.2f (dB)\n", dist_effect );

  // Lombard effect is valid only in this range
  if (noise < 45.0) noise = 45.0;
  if (noise > 70.0) noise = 70.0;

  double noise_effect = ( noise - 45.0 ) * noise_slope;
  //printf( "Noise Effect: %+4.2f (dB)\n", noise_effect );

  double speech_level = init_spl_model + dist_effect + noise_effect;
  //speech_level += gsl_cdf_gaussian_Pinv( double( rand() ) / double( RAND_MAX ), std_dev_model * 0.5 );

  //double speech_level = init_spl + dist_effect + noise_effect;
  //printf( "Speech SPL: %+4.2f (dB) Standard Deviation: %4.2f (dB)\n", speech_level, std_dev );

  return speech_level;
} // calcSplOutB(double, double)

double calcSplInA( double distance,
                   double speaker_orientation,
                   double listener_orientation,
                   double speech_pressure,
                   int    return_code )
{
  return calcSplInB( distance, listener_orientation, speaker_orientation, speech_pressure, return_code );
  return speech_pressure + getDistanceEffect( distance );
} // calcSplInA( double, double, double, double, int )

double calcSplInB( double distance,
                   double speaker_orientation,
                   double listener_orientation,
                   double speech_pressure,
                   int    return_code )
{
  speaker_orientation  = angles::to_degrees( speaker_orientation );
  listener_orientation = angles::to_degrees( listener_orientation );

  double dist  = 0.0;
  double ori_l = 0.0;
  double ori_r = 0.0;
  double spl_l = 0.0;
  double spl_r = 0.0;

  dist = getDistanceEffect( distance );

  // use the data for other ear when the angle is negative
  if ( listener_orientation < 0.0 )
  {
    ori_l = getOrientationEffect( fabs( speaker_orientation ),
                                  fabs( listener_orientation ),
                                  g_model_data_r );
    ori_r = getOrientationEffect( fabs( speaker_orientation ),
                                  fabs( listener_orientation ),
                                  g_model_data_l );
  }
  else
  {
    ori_l = getOrientationEffect( fabs( speaker_orientation ),
                                  fabs( listener_orientation ),
                                  g_model_data_l );
    ori_r = getOrientationEffect( fabs( speaker_orientation ),
                                  fabs( listener_orientation ),
                                  g_model_data_r );
  }
  spl_l = speech_pressure - ori_l + dist;
  spl_r = speech_pressure - ori_r + dist;
/*
  // for left ear
  printf( "Left ear:\n" );
  printf( "Orientation Effect: %+4.2f (dB)\n", -ori_l );
  printf( "Distance Effect: %+4.2f (dB)\n", dist );
  printf( "Sound pressure perceived: %4.2f (dB)\n", spl_l );

  // for right ear
  printf("\nRight ear:\n");
  printf( "Orientation Effect: %+4.2f (dB)\n", -ori_r );
  printf("Distance Effect: %+4.2f (dB)\n", dist);
  printf("Sound pressure perceived: %4.2f (dB)\n", spl_r );
*/
  // here is SPL for both ear of listener:
  //double spl_left  = speech_pressure - ori_l + dist;
  //double spl_right = speech_pressure - ori_r + dist;

  if ( return_code == EAR_LEFT  ) return spl_l;
  if ( return_code == EAR_RIGHT ) return spl_r;
  return std::max( spl_l, spl_r );
} // calcSplInB()

// get the effect of distance on sound pressure
double getDistanceEffect( double dist )
{
  return -20.0 * log10( dist );
} // getDistanceEffect( double )

// get the effect of orientation on sound pressure by bilinear interpolation
double getOrientationEffect( double speaker_orientation,
                             double listener_orientation,
                             float model[ DIRECT_NUM ][ HRTF_NUM ] )
{
  int itx_hrtf1 = 0, itx_hrtf2 = 0, itx_direct = 0, direct_index2 = 0;

  // find the index for interpolation
  direct_index2 = findIndex( speaker_orientation, g_directivity, DIRECT_NUM );
  if (direct_index2 == 0)
  {
    itx_direct    = 0;
    direct_index2 = 1;
  }
  else itx_direct = direct_index2 - 1;

  itx_hrtf2 = findIndex( listener_orientation, g_hrtf, HRTF_NUM );
  if (itx_hrtf2 == 0)
  {
    itx_hrtf1 = 0;
    itx_hrtf2 = 1;
  }
  else itx_hrtf1 = itx_hrtf2 - 1;

  //printf("Index: %d %d\n", itx_hrtf1, itx_hrtf2);

  // bilinear interpolation
  double a = 0.0, b = 0.0;
  a = ( g_directivity[ direct_index2 ] - speaker_orientation ) / ( g_directivity[ direct_index2 ] - g_directivity[ itx_direct ] );
  b = ( g_hrtf[ itx_hrtf2 ] - listener_orientation ) / ( g_hrtf[ itx_hrtf2 ] - g_hrtf[ itx_hrtf1 ] );

  double result = model[ itx_direct    ][ itx_hrtf1] *       a   *       b
                + model[ itx_direct    ][ itx_hrtf2] *       a   * ( 1 - b )
                + model[ direct_index2 ][ itx_hrtf1] * ( 1 - a ) *       b
                + model[ direct_index2 ][ itx_hrtf2] * ( 1 - a ) * ( 1 - b );

  return result;
} // getOrientationEffect(double, double, double)

int findIndex( double data, int index_array[], int array_length )
{
  int i = 0;
  while ( ( data >= index_array[ i ] ) && ( i < array_length ) ) ++i;

  if ( i == 0 )
    return i;
  else if ( data >= index_array[ array_length - 1 ] )
    return array_length - 1;
  return i;
} // findIndex(double, int, int)

void loadModel()
{
  std::string filepath_model_l = g_filepath + MODEL_L;
  std::string filepath_model_r = g_filepath + MODEL_R;
  FILE* file_l = fopen( filepath_model_l.c_str(), "r" );
  FILE* file_r = fopen( filepath_model_l.c_str(), "r" );
  for ( int idx_direct = 0; idx_direct < DIRECT_NUM; ++idx_direct )
    for ( int idx_hrtf = 0; idx_hrtf < HRTF_NUM; ++idx_hrtf )
    {
      fscanf( file_l, "%f", &g_model_data_l[ idx_direct ][ idx_hrtf ] );
      fscanf( file_r, "%f", &g_model_data_r[ idx_direct ][ idx_hrtf ] );
    }
} // loadModel()

// load the performance look-up table for each ear
void loadSpeechRecTables()
{
  std::string filepath_data_dir_perf = g_filepath + DATA_DIR_PERF;
  
  char  file_path_l[ 512 ];
  char  file_path_r[ 512 ];

  for ( int directivity_index = 0; directivity_index < DIRECTIVITY_NUM_PERF; ++directivity_index )
  {
    for (int hrtf_index = 0; hrtf_index < HRTF_NUM_PERF; ++hrtf_index )
    {
      sprintf( file_path_l, "%s/l_ear/speaker_%d_listener_%d.txt", filepath_data_dir_perf.c_str(),
               g_directivity_perf[ directivity_index ], g_hrtf_perf[ hrtf_index ] );
      sprintf( file_path_r, "%s/r_ear/speaker_%d_listener_%d.txt", filepath_data_dir_perf.c_str(),
               g_directivity_perf[ directivity_index ], g_hrtf_perf[ hrtf_index ] );
      FILE* file_l = fopen( file_path_l, "r" );  // note: check for open!!!
      FILE* file_r = fopen( file_path_r, "r" );  // note: check for open!!!

      for ( int speech_pressure = PRESSURE_NUM_PERF - 1; speech_pressure >= 0; --speech_pressure )
        for ( int noise = NOISE_NUM_PERF - 1; noise >= 0; --noise )
        {
          fscanf( file_l, "%f",
                  &g_performance_tables_l[ directivity_index ][ hrtf_index ].performance[ speech_pressure ][ noise ] );
          fscanf( file_r, "%f",
                  &g_performance_tables_r[ directivity_index ][ hrtf_index ].performance[ speech_pressure ][ noise ] );
        }
      fclose( file_l );
      fclose( file_r );
    }
  }
} // loadSpeechRecTables()

// get the indeces for bilinear interpolation
void getTwoSpeechIndeces( float data,   int  index_array[], int array_length,
                          int*  index1, int* index2 )
{
  if ( ( index1 == NULL ) || ( index2 == NULL ) ) return;
  int  temp =  findIndex( data, index_array, array_length );
  if ( temp == 0 )
  {
    *index1 = 0;
    *index2 = 0;
  }
  else
  {
    *index1 = temp - 1;
    *index2 = temp;
  }
} // getTwoSpeechIndeces( float, int[], int, int*, int* )

double bilinearInterpolation( double d11, double d12, double d21, double d22,
                              double a,   double b)
{
  return d11 *         a   *         b
       + d12 *         a   * ( 1.0 - b )
       + d21 * ( 1.0 - a ) *         b
       + d22 * ( 1.0 - a ) * ( 1.0 - b );
} // bilinearInterpolation( double, double, double, double, double, double )

float getSpeechRecRate( TPerformance table[ DIRECTIVITY_NUM_PERF ][ HRTF_NUM_PERF ],
                        double       distance,
                        double       speaker_orientation,
                        double       listener_orientation,
                        double       speech_pressure,
                        double       noise )
{
  speaker_orientation  = angles::to_degrees( speaker_orientation );
  listener_orientation = angles::to_degrees( listener_orientation );
  speech_pressure     += getDistanceEffect( distance );

  int idx_speech1   = 0, idx_speech2   = 0;
  int idx_noise1    = 0, idx_noise2    = 0;
  int idx_speaker1  = 0, idx_speaker2  = 0;
  int idx_listener1 = 0, idx_listener2 = 0;

  // get all the index for interpolation
  getTwoSpeechIndeces( speech_pressure, g_pressure_level_perf, PRESSURE_NUM_PERF,
                      &idx_speech1, &idx_speech2 );
  getTwoSpeechIndeces( noise, g_noise_level_perf, NOISE_NUM_PERF,
                      &idx_noise1, &idx_noise2 );
  getTwoSpeechIndeces( speaker_orientation, g_directivity_perf, DIRECTIVITY_NUM_PERF,
                      &idx_speaker1, &idx_speaker2 );
  getTwoSpeechIndeces( listener_orientation, g_hrtf_perf, HRTF_NUM_PERF,
                      &idx_listener1, &idx_listener2);

  // first, bilinear interpolation for two orientations
  double d11 = 0.0, d12 = 0.0, d21 = 0.0, d22 = 0.0;
  double a   = 0.0, b   = 0.0;
  a   = ( g_directivity_perf[ idx_speaker2 ] - speaker_orientation )
      / ( g_directivity_perf[ idx_speaker2 ] - g_directivity_perf[ idx_speaker1 ] );
  b   = ( g_hrtf_perf[ idx_listener2 ] - listener_orientation )
      / ( g_hrtf_perf[ idx_listener2 ] - g_hrtf_perf[ idx_listener1 ] );
  d11 = bilinearInterpolation(
    table[ idx_speaker1 ][ idx_listener1 ].performance[ idx_speech1 ][ idx_noise1 ],
    table[ idx_speaker1 ][ idx_listener2 ].performance[ idx_speech1 ][ idx_noise1 ],
    table[ idx_speaker2 ][ idx_listener1 ].performance[ idx_speech1 ][ idx_noise1 ],
    table[ idx_speaker2 ][ idx_listener2 ].performance[ idx_speech1 ][ idx_noise1 ],
    a, b );
  d12 = bilinearInterpolation(
    table[ idx_speaker1 ][ idx_listener1 ].performance[ idx_speech1 ][ idx_noise2 ],
    table[ idx_speaker1 ][ idx_listener2 ].performance[ idx_speech1 ][ idx_noise2 ],
    table[ idx_speaker2 ][ idx_listener1 ].performance[ idx_speech1 ][ idx_noise2 ],
    table[ idx_speaker2 ][ idx_listener2 ].performance[ idx_speech1 ][ idx_noise2 ],
    a, b );
  d21 = bilinearInterpolation(
    table[ idx_speaker1 ][ idx_listener1 ].performance[ idx_speech2 ][ idx_noise1 ],
    table[ idx_speaker1 ][ idx_listener2 ].performance[ idx_speech2 ][ idx_noise1 ],
    table[ idx_speaker2 ][ idx_listener1 ].performance[ idx_speech2 ][ idx_noise1 ],
    table[ idx_speaker2 ][ idx_listener2 ].performance[ idx_speech2 ][ idx_noise1 ],
    a, b );
  d22 = bilinearInterpolation(
    table[ idx_speaker1 ][ idx_listener1 ].performance[ idx_speech2 ][ idx_noise2 ],
    table[ idx_speaker1 ][ idx_listener2 ].performance[ idx_speech2 ][ idx_noise2 ],
    table[ idx_speaker2 ][ idx_listener1 ].performance[ idx_speech2 ][ idx_noise2 ],
    table[ idx_speaker2 ][ idx_listener2 ].performance[ idx_speech2 ][ idx_noise2 ],
    a, b );

  // second, bilinear interpolation for speech and noise level
  a = ( g_pressure_level_perf[ idx_speech2 ] - speech_pressure)
    / ( g_pressure_level_perf[ idx_speech2 ] - g_pressure_level_perf[ idx_speech1 ] );
  b = ( g_noise_level_perf[ idx_noise2 ] - noise )
    / ( g_noise_level_perf[ idx_noise2 ] - g_noise_level_perf[ idx_noise1 ] );

  return bilinearInterpolation( d11, d12, d21, d22, a, b );
} // getSpeechRecRate( TPerformance[][], double, double, double, double )

// load all the table
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
