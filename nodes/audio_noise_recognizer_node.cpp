// ROS includes
#include <ros/ros.h>

// ROS dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <proxemics/AudioNoiseConfig.h>

// ROS proxemics includes
#include <proxemics/AudioNoise.h>



// global variables: ROS audio noise messages
proxemics::AudioNoise g_audio_noise;



// callback function prototypes
void cbReconfigure(proxemics::AudioNoiseConfig &config, uint32_t level);



int main(int argc, char** argv)
{

  // initialize ROS
  ros::init(argc, argv, "audio_noise_recognizer");
  ros::NodeHandle nh;

  // create a ROS publisher for audio noise
  // note: currentely, just ambient sound pressure level (SPL)
  ros::Publisher pub_audio_noise = nh.advertise<proxemics::AudioNoise>("audio_noise", 1);

  // initialize dynamic reconfigure parameter server
  dynamic_reconfigure::Server<proxemics::AudioNoiseConfig>               srv_reconfig;
  dynamic_reconfigure::Server<proxemics::AudioNoiseConfig>::CallbackType cb_reconfig;
  cb_reconfig = boost::bind(&cbReconfigure, _1, _2);
  srv_reconfig.setCallback(cb_reconfig);
  
  // initialize parameters
  nh.param("/audio_noise_recognizer/sound_pressure_level", g_audio_noise.sound_pressure_level, 0.0);
  
  //
  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
    pub_audio_noise.publish(g_audio_noise);
	  ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
} // main(int, char**)



void cbReconfigure(proxemics::AudioNoiseConfig &config, uint32_t level)
{
  g_audio_noise.sound_pressure_level = config.sound_pressure_level;
  ROS_INFO("Audio Noise = %.1f dB SPL", g_audio_noise.sound_pressure_level);
} // cbReconfigure(proxemics::AudioNoiseConfig &, uint32_t)
