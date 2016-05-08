/*************************************************************************
* 文件名： tracking
*
* 作者： 冯兵
* 邮件： fengbing123@gmail.com
* 时间： 2016/4/9
*
* 说明： 参考rpg_svo,orb_slam2
*************************************************************************/
#ifndef OPENSLAM_SLAM_TRACKING_H_
#define OPENSLAM_SLAM_TRACKING_H_
#include <openslam/slam/frame.h>
#include <openslam/slam/map.h>

namespace openslam
{
	namespace slam
	{
		/** 跟踪类，主要用于接收帧并计算与之对应的相机姿态，另外决定何时插入一个关键帧，创建一个新
		*   的map point，还有如果跟踪失败执行重定位。
		*/
		class SLAM_IMPEXP TrackingBase
		{
		public:
			/** 跟踪状态
			*/
			enum TrackingState
			{
				TRACKING_SYSTEM_NOT_READY = -1,//状态主要用于导入词汇库的过程
				TRACKING_NO_IMAGES_YET = 0,
				TRACKING_NOT_INITIALIZED = 1,
				TRACKING_OK = 2,
				TRACKING_LOST = 3
			};

			TrackingBase(ORBVocabulary *voc);
			virtual ~TrackingBase();
			/**不同的传感器初始化不一致*/
			virtual void initialize() = 0;

			/**输入一幅图像，给单目的接口*/
			virtual void addImage(const cv::Mat& img, double timestamp) = 0;
		protected:
			TrackingState tracking_state_;//!<表示设置的跟踪状态
			Map  *map_;
			ORBVocabulary *orb_vocabulary_;
			bool is_only_tracking_;//!< 如果是true,则表明只执行定位
			bool is_vo_;
			cv::Mat motion_model_;

		};
	}
}

#endif // OPENSLAM_SLAM_TRACKING_H_