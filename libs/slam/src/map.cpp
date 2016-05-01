#include <openslam/slam/map.h>

namespace openslam
{
	namespace slam
	{
		Map::Map() :max_key_frame_id_(0)
		{
		}

		Map::~Map()
		{
		}

		void Map::addKeyframe(KeyFramePtr new_keyframe)
		{
			std::unique_lock<std::mutex> lock(mutex_map_);
			list_keyframes_.push_back(new_keyframe);
			if (new_keyframe->id_ > max_key_frame_id_)
				max_key_frame_id_ = new_keyframe->id_;
		}
	}
}