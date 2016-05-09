#ifndef OPENSLAM_SLAM_TRACKING_MONO_H_
#define OPENSLAM_SLAM_TRACKING_MONO_H_
#include <openslam/slam/tracking_base.h>
#include <openslam/slam/initializer.h>
#include <openslam/slam/monocular_camera.h>
#include <openslam/slam/map.h>

namespace openslam
{
	namespace slam
	{
		class SLAM_IMPEXP TrackingMono : public TrackingBase
		{
		public:
			enum Stage {
				STAGE_PAUSED,//暂停
				STAGE_FIRST_FRAME,//第一帧
				STAGE_SECOND_FRAME,//第二帧
				STAGE_DEFAULT_FRAME,//默认帧
				STAGE_RELOCALIZING//重新定位
			};

			/**根据配置文件进行初始化*/
			TrackingMono(const std::string &setting_path_name, ORBVocabulary * voc);
			/**继续为虚函数，可以进一步扩展*/
			virtual ~TrackingMono();

			virtual void initialize();
			/**输入一幅图像*/
			void addImage(const cv::Mat& img, double timestamp);

		protected:
			/**处理第一帧*/
			virtual bool processFirstFrame();

			/**处理接下来的所有帧，直到下一个关键帧被选择*/
			virtual bool processSecondFrame();

			/**处理完开始的两个关键帧后处理所有帧*/
			virtual bool processFrame();

		protected:
			Stage stage_;                 //!< 算法当前所处的阶段,单目内部状态
			MonocularCamera *cam_;//!<单目相机
			ORBextractor* init_orb_extractor_;//!<初始化的时候特征提取
			ORBextractor* extractor_;//!< orb特征提取
			Initializer initializer_;//!<主要用于单目slam 的初始化
			
		};
	}
}

#endif // OPENSLAM_SLAM_TRACKING_MONO_H_
