/*************************************************************************
* 文件名： frame
*
* 作者： 冯兵
* 邮件： fengbing123@gmail.com
* 时间： 2016/4/9
*
* 说明： 对特征进行简单封装
*************************************************************************/
#ifndef OPENSLAM_SLAM_FEATURE_H_
#define OPENSLAM_SLAM_FEATURE_H_
#include <openslam/slam/frame.h>
#include <openslam/slam/map_point.h>

namespace openslam
{
	namespace slam
	{
		class SLAM_IMPEXP Feature
		{
		public:
			Feature(Frame* frame, const cv::KeyPoint& keypoint, cv::Mat &descriptor);
			~Feature();
			/** 添加特征对应的map point
			*/
			void addMapPointRef(MapPoint *map_point);

			/** 添加特征所对应的帧
			*/
			void addFrameRef(Frame *frame);

		private:
			void undistortKeyPoint();
		public:
			Frame*         frame_;              //!< 指针指向特征被检测到所对应的帧
			cv::KeyPoint   keypoint_;           //!< 每个特征对应的特征点
			cv::KeyPoint   undistored_keypoint_;//!< 进行畸变矫正之后的特征点
			cv::Mat        descriptor_;         //!< 每个特征对应的特征描述
			MapPoint*      map_point_;          //!< 指针指向跟特征对应的3D点
		
		};
	}
}

#endif // OPENSLAM_SLAM_FEATURE_H_
