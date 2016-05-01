#ifndef OPENSLAM_SLAM_MAP_H_
#define OPENSLAM_SLAM_MAP_H_
#include <openslam/slam/link_pragmas.h>
#include <mutex>
#include <openslam/slam/keyframe.h>
#include <openslam/slam/map_point.h>

namespace openslam
{
	namespace slam
	{
		class SLAM_IMPEXP Map
		{
		public:
			Map();
			~Map();
			/**往地图中添加一个新的关键帧*/
			void addKeyframe(KeyFramePtr new_keyframe);

		protected:
			std::list< KeyFramePtr > list_keyframes_;          //!< 地图中存储的所有关键帧
			std::list< MapPoint * > list_map_points_;//!< 地图中存储的MapPoint
			long unsigned int max_key_frame_id_;//!< 用于记录地图中关键帧的最大id
			std::mutex mutex_map_;//!<map 更新用的锁
		};
	}
}

#endif // OPENSLAM_SLAM_MAP_H_
