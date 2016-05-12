#ifndef OPENSLAM_SLAM_ORB_MATCHER_H_
#define OPENSLAM_SLAM_ORB_MATCHER_H_
#include <openslam/slam/frame.h>
#include <opencv2/opencv.hpp>

namespace openslam
{
	namespace slam
	{
		class MapPoint;
		class SLAM_IMPEXP ORBmatcher
		{
		public:

			ORBmatcher(float nnratio = 0.6, bool check_orientation = true);

			~ORBmatcher();
			/** 计算两个特征之间的汉明距离
			* 具体计算参考 http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
			*/
			static int descriptorDistance(const cv::Mat &a, const cv::Mat &b);

			/** 两帧之间寻找匹配，用于单目的初始化
			*/
			int searchForInitialization(FramePtr ref_frame, FramePtr cur_frame,
				std::vector<cv::Point2f> &prev_matched, std::vector<int> &matches_ref_cur, int window_size = 10);

			/** 将上一帧中获得的mappoint点投影到当前帧中，进行匹配
			*/
			int searchByProjection(FramePtr cur_frame, const FramePtr last_frame, const float th);

			/**
			*/
			int searchByBoW(KeyFrame *keyframe, FramePtr frame);
		protected:
			/** \brief 计算直方图中高度前3的柱体的索引
			*/
			void computeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

		public:
			static const int TH_LOW;
			static const int TH_HIGH;
			static const int HISTO_LENGTH;//!<检查特征方向是否一致，直方图分组个数，默认30

		protected:
			float nn_ratio_;
			bool is_check_orientation_;

		};
	}
}

#endif // OPENSLAM_SLAM_ORB_MATCHER_H_
