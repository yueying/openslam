#ifndef OPENSLAM_SLAM_LOCAL_MAPPING_H_
#define OPENSLAM_SLAM_LOCAL_MAPPING_H_

#include <queue>
#include <thread>
#include <openslam/slam/map.h>
#include <openslam/slam/keyframe.h>

namespace openslam
{
	namespace slam
	{
		class SLAM_IMPEXP LocalMapping
		{
		public:
			/**传入地图进行优化*/
			LocalMapping(Map* map);

			virtual ~LocalMapping();

			/**启动线程*/
			void startThread();

			/**关闭线程*/
			void stopThread();

			/** 插入关键帧，以便下一步进行局部地图优化
			*/
			void insertKeyFrame(KeyFramePtr keyframe);
		protected:
			void run();

		protected:
			bool local_mapping_stop_;//!<线程关闭结束标识
			std::thread* thread_;//!<自己线程
			std::mutex mutex_new_keyframes_;//!<添加关键帧锁
			bool is_runing_;//!<用于线程是否结束
			Map* map_;
			std::queue<KeyFramePtr> list_new_keyframes_;//!<存储关键帧列表，主要进行局部地图优化
		};
	}
}

#endif // OPENSLAM_SLAM_LOCAL_MAPPING_H_
