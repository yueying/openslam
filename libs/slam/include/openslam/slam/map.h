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
			void addKeyframe(KeyFrame * new_keyframe);

			/** \brief 地图中删除关键帧
			*/
			void eraseKeyFrame(KeyFrame * keyframe);

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
			std::vector<KeyFrame *> getAllKeyFrames();

			/** \brief 得到地图中的所有map point
			*/
			std::vector<MapPoint*> getAllMapPoints();

			/** \brief 将上一帧计算的map point做为这一帧的参考
			*/
			void setReferenceMapPoints(const std::vector<MapPoint*> &vec_map_points);

			/***/
			void getVectorCovisibleKeyFrames(const KeyFrame *frame, std::vector<KeyFrame*> &vec_neigh_keyframes);

		public:
			std::mutex mutex_map_update_;
		protected:
			std::set< KeyFrame * > set_keyframes_;          //!< 地图中存储的所有关键帧
			std::set< MapPoint * > set_map_points_;//!< 地图中存储的MapPoint
			std::vector<MapPoint*> vec_ref_map_points_;//!<将上一帧的map point保存进行参考，主要进行可视化操作
			long unsigned int max_key_frame_id_;//!< 用于记录地图中关键帧的最大id
			std::mutex mutex_map_;//!<map 更新用的锁
		};
	}
}

#endif // OPENSLAM_SLAM_MAP_H_
