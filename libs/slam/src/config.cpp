#include <openslam/slam/config.h>

namespace openslam
{
	namespace slam
	{
		Config::Config() :
			sensor_(SENSOR_MONOCULAR)
		{
		}

		Config& Config::getInstance()
		{
			static Config instance; 
			return instance;
		}
	}
}