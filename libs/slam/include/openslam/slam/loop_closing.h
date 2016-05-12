#ifndef OPENSLAM_SLAM_LOOP_CLOSING_H_
#define OPENSLAM_SLAM_LOOP_CLOSING_H_


#include <queue>
#include <thread>
#include <openslam/slam/map.h>
#include <openslam/slam/keyframe.h>

namespace openslam
{
	namespace slam
	{
		class SLAM_IMPEXP LoopClosing
		{
		public:
			/**传入地图进行优化*/
			LoopClosing(Map* map);

			virtual ~LoopClosing();

			/**启动线程*/
			void startThread();

			/**关闭线程*/
			void stopThread();

			/** 插入关键帧
			*/
			void insertKeyFrame(KeyFrame * keyframe);
		protected:
			void run();

		protected:
			bool loop_closing_stop_;//!<线程关闭结束标识
			std::thread* thread_;//!<自己线程
			std::mutex mutex_new_keyframes_;//!<添加关键帧锁
			bool is_runing_;//!<用于线程是否结束
			Map* map_;
			std::queue<KeyFrame *> queue_new_keyframes_;//!<存储关键帧列表，主要进行局部地图优化
		};
	}
}



#endif // OPENSLAM_SLAM_LOOP_CLOSING_H_
