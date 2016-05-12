#ifndef OPENSLAM_SLAM_KEYFRAME_H_
#define OPENSLAM_SLAM_KEYFRAME_H_

#include <openslam/slam/frame.h>
#include <mutex>

namespace openslam
{
	namespace slam
	{
		class MapPoint;
	
		class SLAM_IMPEXP KeyFrame : public Frame
		{
		public:
			KeyFrame(const Frame &frame);
			~KeyFrame();

			/** \brief 计算深度，转到当前帧的坐标系下，计算所有特征对应的map point的z的中值，即为深度值
			*/
			float computeSceneMedianDepth();

			/** \brief 用于计算该关键帧上所有特征对应的map point被观察到的其它关键帧个数
			* 两个关键帧至少有\param min_obs 共同map point
			*/
			int trackedMapPoints(const int &min_obs);

			/** \brief 设置关键帧是否可用的标识
			*/
			void setBadFlag();

			/** \brief 得到关键帧是否可用
			*/
			bool isBad();

			/** \brief 关键帧上特征点对应所有的map point
			*/
			std::vector<MapPoint*> getMapPointMatches();
		public:
			static long unsigned int keyframe_counter_;//!< 创建关键帧的计数器，用于设置帧的唯一id
			long unsigned int keyframe_id_;//!< 关键帧的唯一id
			cv::Mat gba_Tcw_;
			long unsigned int gba_for_keyframe_num_;
			long unsigned int local_ba_for_keyframe_id_;
			long unsigned int fixed_ba_for_keyframe_id_;
		protected:
			std::mutex mutex_pose_;
			std::mutex mutex_features_;
			std::mutex mutex_connections_;
			bool is_bad_;

		};
	}
}

#endif // OPENSLAM_SLAM_KEYFRAME_H_
