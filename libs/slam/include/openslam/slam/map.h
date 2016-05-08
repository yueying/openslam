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

			/** \brief 地图中删除关键帧
			*/
			void eraseKeyFrame(KeyFramePtr keyframe);

			/** \brief 地图中添加map point
			*/
			void addMapPoint(MapPoint* map_point);

			/** \brief 地图中删除 map point
			*/
			void eraseMapPoint(MapPoint* map_point);

			/** \brief 地图中存储的map point的个数
			*/
			long unsigned int mapPointsInMap();

			/** \brief 地图中存储的关键帧的个数
			*/
			long unsigned int keyFramesInMap();

			/** \brief 得到地图中的所有关键帧
			*/
			std::vector<KeyFramePtr> getAllKeyFrames();

			/** \brief 得到地图中的所有map point
			*/
			std::vector<MapPoint*> getAllMapPoints();
		protected:
			std::set< KeyFramePtr > set_keyframes_;          //!< 地图中存储的所有关键帧
			std::set< MapPoint * > set_map_points_;//!< 地图中存储的MapPoint
			long unsigned int max_key_frame_id_;//!< 用于记录地图中关键帧的最大id
			std::mutex mutex_map_;//!<map 更新用的锁
		};
	}
}

#endif // OPENSLAM_SLAM_MAP_H_
