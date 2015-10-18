// standard includes
#include <fcntl.h>
#include <unistd.h>

// ALSA sound mixer includes
#include <alsa/asoundlib.h>

// ROS includes
#include <ros/ros.h>

// ROS dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <proxemics/VocalicsControllerConfig.h>

// ROS proxemics includes
#include <proxemics/VocalicsGoalState.h>



struct SoundMixer
{
  snd_mixer_t*          handle;
  snd_mixer_elem_t*     elem;
  snd_mixer_selem_id_t* sid;
  bool                  is_open;


  
  SoundMixer(): handle(NULL), elem(NULL), sid(NULL), is_open(false)
  {
  } // SoundMixer()



  bool open(const char* mix_name = "Master", const char* card = "default", const int mix_index = 0)
  {
	close();
	
    snd_mixer_selem_id_alloca(&sid);

    // sets simple-mixer index and name
    snd_mixer_selem_id_set_index(sid, mix_index);
    snd_mixer_selem_id_set_name( sid, mix_name);

    is_open = (snd_mixer_open(&handle, 0) == 0);
    if (!isOpen())
    {
	  return false;
	}
    
    if ((snd_mixer_attach(handle, card)) < 0)
    {
      close();
      return false;
    }
    
    if ((snd_mixer_selem_register(handle, NULL, NULL)) < 0)
    {
      close();
      return false;
    }
    
    if (snd_mixer_load(handle) < 0)
    {
      close();
      return false;
    }
    
    elem = snd_mixer_find_selem(handle, sid);
    if (!elem)
    {
      close();
      return false;
    }
    
    return true;
  } // open(const char*, const char*, const int)
  
  
  
  bool isOpen()
  {
	return is_open;
  } // isOpen()
  
  
  
  bool close()
  {
	if (isOpen())
	{
		snd_mixer_close(handle);
		handle  = NULL;
		elem    = NULL;
		sid     = NULL;
		is_open = false;
	}
	return true;
  } // close()
  
  
  
  bool setVolume(double volume)
  {
	if (!isOpen()) return false;
	
	if (volume <   0.0) volume =   0.0;
	if (volume > 100.0) volume = 100.0;
    
    long min_volume = -1, max_volume = -1;
    getVolumeRanges(min_volume, max_volume);
    
    long vol = min_volume + (max_volume - min_volume) * volume / 100.0;
    return snd_mixer_selem_set_playback_volume_all(elem, vol) == 0;
  } // setVolume(long)
  
  
  
  double getVolume()
  {
	if (!isOpen()) return false;
	
	long volume = -1;
	if (snd_mixer_selem_get_playback_volume(elem, SND_MIXER_SCHN_MONO, &volume) < 0)
	{
      close();
      return false;
    }
    
    long min_volume = -1, max_volume = -1;
    getVolumeRanges(min_volume, max_volume);
    
    // bound the value of volume from 0 to 100
    return 100.0 * (volume - min_volume) / (max_volume - min_volume);
  } // getVolume(long &)
  
  
  
  bool getVolumeRanges(long &min_volume, long &max_volume)
  {
	if (!isOpen()) return false;
	snd_mixer_selem_get_playback_volume_range (elem, &min_volume, &max_volume);
	return true;
  } // getVolumeRanges(long &, long &)
  
  
  
  bool getVolumeMin(long &min_volume)
  {
	long max_volume;
	return getVolumeRanges(min_volume, max_volume);
  } // getVolumeMin()
  
  
  
  bool getVolumeMax(long &max_volume)
  {
	long min_volume;
	return getVolumeRanges(min_volume, max_volume);
  } // getVolumeMax()
};// SoundMixer



// global variables
bool   g_set_volume          = true;
double g_volume              = 0.0;
double g_spl_to_volume_beta0 = 0.0;
double g_spl_to_volume_beta1 = 0.0;
double g_spl_to_volume_beta2 = 0.0;
double g_volume_to_spl_beta0 = 0.0;
double g_volume_to_spl_beta1 = 0.0;
double g_volume_to_spl_beta2 = 0.0;



// callback function prototypes
void cbVocalicsGoalState(const proxemics::VocalicsGoalState::ConstPtr &vocalics_goal_state);
void cbReconfigure(proxemics::VocalicsControllerConfig &config, uint32_t level);



// dB SPL / volume conversion function prototypes
double convertSplToVolume(double spl);
double convertVolumeToSpl(double volume);



int main(int argc, char** argv)
{

  // initialize ROS
  ros::init(argc, argv, "vocalics_controller");
  ros::NodeHandle nh;
  
  //
  SoundMixer sound_mixer;
  if (!sound_mixer.open())
  {
	  ROS_ERROR("Failed to open sound mixer...");
	  exit(1);
  }
  
  // initialize subscribers
  ros::Subscriber sub = nh.subscribe("vocalics_goal_state", 1, cbVocalicsGoalState);

  // initialize dynamic reconfigure parameter server
  dynamic_reconfigure::Server<proxemics::VocalicsControllerConfig> srv_reconfig;
  dynamic_reconfigure::Server<proxemics::VocalicsControllerConfig>::CallbackType cb_reconfig;
  cb_reconfig = boost::bind(&cbReconfigure, _1, _2);
  srv_reconfig.setCallback(cb_reconfig);
  
  // initiatlize parameters
  nh.param("spl_to_volume_beta0", g_spl_to_volume_beta0, 0.0);
  nh.param("spl_to_volume_beta1", g_spl_to_volume_beta1, 0.0);
  nh.param("spl_to_volume_beta2", g_spl_to_volume_beta2, 0.0);
  nh.param("volume_to_spl_beta0", g_volume_to_spl_beta0, 0.0);
  nh.param("volume_to_spl_beta1", g_volume_to_spl_beta1, 0.0);
  nh.param("volume_to_spl_beta2", g_volume_to_spl_beta2, 0.0);
  
  //
  ros::Rate loop_rate(10);
  
  double curr_volume = sound_mixer.getVolume(), prev_volume = curr_volume;
  
  while (ros::ok())
  {
	if (g_set_volume)
	{
		prev_volume  = sound_mixer.getVolume();
		sound_mixer.setVolume(g_volume);
		curr_volume  = sound_mixer.getVolume();
		g_set_volume = false;
        ROS_INFO("Volume = %.1lf\% (%.1lf dB SPL).",
                 curr_volume, convertVolumeToSpl(curr_volume));
        //ROS_INFO("Volume = %.1lf\% (was %.1lf\%).",
        //         curr_volume, prev_volume);
	}
	ros::spinOnce();
    loop_rate.sleep();
  }
  
  sound_mixer.close();
  
  return 0;
} // main(int, char**)



void cbVocalicsGoalState(const proxemics::VocalicsGoalState::ConstPtr &vocalics_goal_state)
{
  g_volume     = convertSplToVolume(vocalics_goal_state->sound_pressure_level);
  g_set_volume = true;
} // cbVocalicsGoalState(const proxemics::VocalicsGoalState::ConstPtr &)



void cbReconfigure(proxemics::VocalicsControllerConfig &config, uint32_t level)
{
  g_volume              = config.volume;
  g_spl_to_volume_beta0 = config.spl_to_volume_beta0;
  g_spl_to_volume_beta1 = config.spl_to_volume_beta1;
  g_spl_to_volume_beta2 = config.spl_to_volume_beta2;
  g_volume_to_spl_beta0 = config.volume_to_spl_beta0;
  g_volume_to_spl_beta1 = config.volume_to_spl_beta1;
  g_volume_to_spl_beta2 = config.volume_to_spl_beta2;
  g_set_volume = true;
} // cbReconfigure(proxemics::VocalicsControllerConfig &, uint32_t)



double convertSplToVolume(double spl)
{
  double volume = g_spl_to_volume_beta2 * pow(spl, 2) 
                + g_spl_to_volume_beta1 * spl 
                + g_spl_to_volume_beta0;
  if (volume < 0.0) volume = 0.0;
  return volume;
} // convertSplToVolume(double)



double convertVolumeToSpl(double volume)
{
  double spl = g_volume_to_spl_beta2 * pow(volume, 2) 
             + g_volume_to_spl_beta1 * volume 
             + g_volume_to_spl_beta0;
  return spl;
} // convertVolumeToSpl(double)
