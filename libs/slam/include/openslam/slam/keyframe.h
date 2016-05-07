#ifndef OPENSLAM_SLAM_KEYFRAME_H_
#define OPENSLAM_SLAM_KEYFRAME_H_

#include <openslam/slam/frame.h>

namespace openslam
{
	namespace slam
	{
		class SLAM_IMPEXP KeyFrame : public Frame
		{
		public:
			KeyFrame(const Frame &frame);
			~KeyFrame();

		public:
			static long unsigned int keyframe_counter_;//!< 创建关键帧的计数器，用于设置帧的唯一id
			long unsigned int keyframe_id_;//!< 关键帧的唯一id

		};

		typedef std::shared_ptr<KeyFrame> KeyFramePtr;
	}
}

#endif // OPENSLAM_SLAM_KEYFRAME_H_
