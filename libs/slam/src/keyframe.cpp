#include <openslam/slam/keyframe.h>

namespace openslam
{
	namespace slam
	{
		long unsigned int KeyFrame::keyframe_counter_ = 0;
		KeyFrame::KeyFrame(const Frame &frame) :Frame(frame)
		{
			keyframe_id_ = keyframe_counter_++;
		}

		KeyFrame::~KeyFrame()
		{

		}
	}
}