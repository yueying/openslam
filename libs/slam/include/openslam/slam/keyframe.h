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
			KeyFrame();
			~KeyFrame();

		public:


		};

		typedef std::shared_ptr<KeyFrame> KeyFramePtr;
	}
}

#endif // OPENSLAM_SLAM_KEYFRAME_H_
