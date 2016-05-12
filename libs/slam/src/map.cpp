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

		void Map::addKeyframe(KeyFrame * new_keyframe)
		{
			std::unique_lock<std::mutex> lock(mutex_map_);
			set_keyframes_.insert(new_keyframe);
			if (new_keyframe->id_ > max_key_frame_id_)
				max_key_frame_id_ = new_keyframe->id_;
		}

		void Map::eraseKeyFrame(KeyFrame * keyframe)
		{
			std::unique_lock<mutex> lock(mutex_map_);
			set_keyframes_.erase(keyframe);

			// TODO: This only erase the pointer.
			// Delete the MapPoint
		}

		void Map::addMapPoint(MapPoint *map_point)
		{
			std::unique_lock<mutex> lock(mutex_map_);
			set_map_points_.insert(map_point);
		}

		void Map::eraseMapPoint(MapPoint *map_point)
		{
			std::unique_lock<mutex> lock(mutex_map_);
			set_map_points_.erase(map_point);

			// TODO: This only erase the pointer.
			// Delete the MapPoint
		}

		long unsigned int Map::mapPointsInMap()
		{
			std::unique_lock<mutex> lock(mutex_map_);
			return set_map_points_.size();
		}

		long unsigned int Map::keyFramesInMap()
		{
			std::unique_lock<mutex> lock(mutex_map_);
			return set_keyframes_.size();
		}

		std::vector<KeyFrame *> Map::getAllKeyFrames()
		{
			std::unique_lock<mutex> lock(mutex_map_);
			return std::vector<KeyFrame *>(set_keyframes_.begin(), set_keyframes_.end());
		}

		std::vector<MapPoint*> Map::getAllMapPoints()
		{
			std::unique_lock<mutex> lock(mutex_map_);
			return std::vector<MapPoint*>(set_map_points_.begin(), set_map_points_.end());
		}

		void Map::setReferenceMapPoints(const std::vector<MapPoint *> &vec_map_points)
		{
			std::unique_lock<mutex> lock(mutex_map_);
			vec_ref_map_points_ = vec_map_points;
		}

		void Map::getVectorCovisibleKeyFrames(const KeyFrame *frame, std::vector<KeyFrame*> &vec_neigh_keyframes)
		{

		}
	}
}