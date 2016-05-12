#include <openslam/slam/system.h>
#include <openslam/utils/notify.h>
#include <openslam/utils/timer.h>
#include <openslam/slam/tracking_mono.h>
#include <openslam/slam/config.h>

namespace openslam
{
	namespace slam
	{
		System::System(const std::string &voc_file_name, const std::string &settings_file_name
			, const bool is_use_viewer)
		{
			if (Config::sensorType() == Config::SENSOR_MONOCULAR)
				OPENSLAM_INFO << "Monocular" << std::endl;
			else if (Config::sensorType() == Config::SENSOR_STEREO)
				OPENSLAM_INFO << "Stereo" << std::endl;
			else if (Config::sensorType() == Config::SENSOR_RGBD)
				OPENSLAM_INFO << "RGB-D" << std::endl;
			//导入ORB的词汇表
			utils::Timer timer;
			std::cout << std::endl << "Loading ORB Vocabulary. This could take a while..." << std::endl;

			orb_vocabulary_ = new ORBVocabulary();
			bool is_voc_load = false; // 这边为了导入词汇表的时间快，可直接导入二进制流
			std::string suffix = ".txt";
			std::size_t index = voc_file_name.find(suffix, voc_file_name.size() - suffix.size());
			if (index != std::string::npos)
				is_voc_load = orb_vocabulary_->loadFromTextFile(voc_file_name);
			else
				is_voc_load = orb_vocabulary_->loadFromBinaryFile(voc_file_name);
			if (!is_voc_load)
			{
				OPENSLAM_FATAL << "Wrong path to vocabulary. " << std::endl;
				OPENSLAM_FATAL << "Falied to open at: " << voc_file_name << std::endl;
				exit(-1);
			}
			OPENSLAM_INFO << "Vocabulary loaded in " << timer.Stop() << "s" << std::endl;

			//根据不同的传感器实例化不同的类
			switch (Config::sensorType())
			{
			case Config::SENSOR_MONOCULAR:
				tracker_ = new TrackingMono(settings_file_name, orb_vocabulary_);
				break;
			case Config::SENSOR_STEREO:
				break;
			case Config::SENSOR_RGBD:
				break;
			default:
				break;
			}
		}
		System::~System()
		{

		}

		void System::addImage(const cv::Mat& img, double timestamp)
		{
			tracker_->addImage(img, timestamp);
		}
	}
}