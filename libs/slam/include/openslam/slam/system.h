#ifndef OPENSLAM_SLAM_SYSTEM_MONO_H_
#define OPENSLAM_SLAM_SYSTEM_MONO_H_
#include <openslam/slam/map.h>
#include <openslam/slam/tracking_base.h>

namespace openslam
{
	namespace slam
	{
		/**给用户接口，用户就是提供图像，然后得到实时相机位姿，轨迹数据，地图数据*/
		class SLAM_IMPEXP System
		{
		public:
			
			System(const std::string &voc_file_name, const std::string &settings_file_name, const bool is_use_viewer = true);
			~System();

			/**输入图像，得到结果*/
			void addImage(const cv::Mat& img, double timestamp);

			
		protected:

			ORBVocabulary* orb_vocabulary_;//!< 放在这边用于导入词汇表
			TrackingBase* tracker_;//!<跟踪的基类，根据不同的传感器，跟踪不同
			Map *map_;//!<地图信息


		};
	}
}

#endif // OPENSLAM_SLAM_SYSTEM_MONO_H_
