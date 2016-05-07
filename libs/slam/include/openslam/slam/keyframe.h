#ifndef OPENSLAM_SLAM_KEYFRAME_H_
#define OPENSLAM_SLAM_KEYFRAME_H_

#include <openslam/slam/frame.h>
#include <mutex>

namespace openslam
{
	namespace slam
	{
		class SLAM_IMPEXP KeyFrame : public Frame
		{
		public:
			KeyFrame(const Frame &frame);
			~KeyFrame();

			/** \brief 计算深度，转到当前帧的坐标系下，计算所有特征对应的map point的z的中值，即为深度值
			*/
			float computeSceneMedianDepth();
		public:
			static long unsigned int keyframe_counter_;//!< 创建关键帧的计数器，用于设置帧的唯一id
			long unsigned int keyframe_id_;//!< 关键帧的唯一id

		protected:
			std::mutex mutex_pose_;
			std::mutex mutex_features_;

		};

		typedef std::shared_ptr<KeyFrame> KeyFramePtr;
	}
}

#endif // OPENSLAM_SLAM_KEYFRAME_H_
