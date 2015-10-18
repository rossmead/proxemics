// GSL includes
//#include <gsl/gsl_cdf.h>
//#include <gsl/gsl_randist.h>

// ROS includes
#include <ros/package.h>  // for finding the filepath to the package
#include <ros/ros.h>
#include <tf/transform_listener.h>

// ROS proxemics includes
#include <proxemics/AudioNoise.h>
#include <proxemics/VocalicsGoalState.h>
#include <proxemics/KinesicsGoalState.h>

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

// function prototypes: callbacks
void cbAudioNoise(const proxemics::AudioNoise::ConstPtr &audio_noise);

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



// global variables: ROS proxemics messages
proxemics::VocalicsGoalState g_vocalics_goal_state;

// global variables: audio noise
double g_ambient_spl = 45.0;

// executes main program code
int main( int argc, char** argv )
{
  // Initialize ROS.
  ros::init( argc, argv, "proxemics_goal_state_estimation" );
  ros::NodeHandle nh;

  // Create a ROS publisher for the vocalics goal state.
  ros::Publisher pub_vocalics_goal_state  = nh.advertise<proxemics::VocalicsGoalState>(  "vocalics_goal_state",  1 );
  
  // Create a ROS subscriber for the audio noise.
  ros::Subscriber sub = nh.subscribe("audio_noise", 1, cbAudioNoise);

  // Load speech recognition tables.
  nh.param<std::string>("/proxemics_goal_state_estimation/filepath", g_filepath, ros::package::getPath("proxemics"));
  loadSpeechRecTables();
  
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
        
        g_vocalics_goal_state.sound_pressure_level = calcSplOutA( 
          g_curr_range_robot_to_human, 
          g_curr_angle_robot_to_human, 
          g_curr_angle_human_to_robot,
          g_ambient_spl );
      }
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
    }
    
    pub_vocalics_goal_state.publish( g_vocalics_goal_state );

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
} // main( int, char** )

// callback function: audio noise
void cbAudioNoise(const proxemics::AudioNoise::ConstPtr &audio_noise)
{
  g_ambient_spl = audio_noise->sound_pressure_level;
} // cbAudioNoise(const proxemics::AudioNoise::ConstPtr &)

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

  const double proxemics_spl_out_b_mean    = 67.23;  // proxemics::spl_out_b_mean from proxemics_goal_state_estimation_node
  const double proxemics_spl_out_b_std_dev = 5.90;   // proxemics::spl_out_b_std_dev from proxemics_goal_state_estimation_node
  const double init_spl_model      = proxemics_spl_out_b_mean;      // proxemics::spl_out_b_mean;
  //const double std_dev_model       = proxemics_spl_out_b_std_dev;   // proxemics::spl_out_b_std_dev;
  const double dist_slope_model_hh = ( 66.37 - 63.63 ) / ( 5.0 - 0.5 );
  //const double dist_slope_model_hr = ( 65.78 - 75.20 ) / ( 5.0 - 0.5 );
  const double dist_slope_model    = dist_slope_model_hh;

  double dist_effect = 0.0;
  //if (dist  <  0.3) dist  = 0.3;  // 0.3m is the minimum distance allowed
  //if ( dist <= dist_close )
  //  dist_effect += log2( dist / 1.0 ) * dist_slope_close;
  //else
  //  dist_effect += log2( 1.5 / 1.0 ) * dist_slope + log2( dist / 1.5 ) * dist_slope;

  //dist       += gsl_cdf_gaussian_Pinv( double( rand() ) / double( RAND_MAX ), proxemics::rng_std_dev );
  double proxemics_rng_mean = 1.65;  // proxemics::rng_mean from proxemics_goal_state_estimation_node
  dist_effect = ( dist - proxemics_rng_mean ) * dist_slope_model;
  //dist_effect = ( dist - proxemics::rng_mean ) * dist_slope_model;
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
  
  // clip sound pressure value to those represented in the data collection
  if ( speech_pressure < g_pressure_level_perf[ 0 ] )
    speech_pressure = g_pressure_level_perf[ 0 ];
  if ( speech_pressure > g_pressure_level_perf[ PRESSURE_NUM_PERF - 1 ] )
    speech_pressure = g_pressure_level_perf[ PRESSURE_NUM_PERF - 1 ];
  
  // clip noise value to those represented in the data collection
  if ( noise < g_noise_level_perf[ 0 ] )
    noise = g_noise_level_perf[ 0 ];
  if ( noise > g_noise_level_perf[ NOISE_NUM_PERF - 1 ] )
    noise = g_noise_level_perf[ NOISE_NUM_PERF - 1 ];

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
