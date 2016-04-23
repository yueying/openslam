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

namespace openslam
{
	namespace slam
	{
		class SLAM_IMPEXP Tracking
		{
		public:
			/** 跟踪状态
			*/
			enum TrackingState
			{
				TRACKING_SYSTEM_NOT_READY = -1,
				TRACKING_NO_IMAGES_YET = 0,
				TRACKING_NOT_INITIALIZED = 1,
				TRACKING_OK = 2,
				TRACKING_LOST = 3
			};
		};
	}
}

#endif // OPENSLAM_SLAM_TRACKING_H_