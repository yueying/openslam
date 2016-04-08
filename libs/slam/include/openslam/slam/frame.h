/*************************************************************************
* 文件名： frame
*
* 作者： 冯兵
* 邮件： fengbing123@gmail.com
* 时间： 2016/4/9
*
* 说明： 参考rpg_svo,orb_slam2
*************************************************************************/
#ifndef OPENSLAM_SLAM_FRAME_H_
#define OPENSLAM_SLAM_FRAME_H_
#include <openslam/slam/link_pragmas.h>
#include <vector>
#include <opencv2/opencv.hpp>

#include <openslam/slam/pinhole_camera.h>
#include <openslam/slam/orb_extractor.h>

namespace openslam
{
	namespace slam
	{
		class Feature;

		typedef std::vector<Feature> Features;

		class SLAM_IMPEXP Frame
		{
		public:
			/** 帧的构造，给出对应相机参数及对应原始帧及对应的时间码
			*/
			Frame(PinholeCamera* cam, const cv::Mat& img, double timestamp,ORBextractor* extractor);
			~Frame();

		private:
			/** \brief 对图像进行ORB特征检测
			*/
			void extractORB(const cv::Mat &image);

		public:
			static long unsigned int     frame_counter_;//!< 创建帧的计数器，用于设置帧的唯一id
			long unsigned int            id_;           //!< 帧的id
			double                       timestamp_;    //!< 帧的时间戳
			PinholeCamera*               cam_;          //!< 相机模型
			cv::Mat                      img_;          //!< 帧对应的原始图像
			float                        scale_factor_; //!< 对应金字塔图像的尺度因子
			int                          levels_num_;   //!< 对应金字塔的层数
			Features                     features_;     //!< 帧对应的特征
			int                          keypoints_num_;//!< 特征点的个数
			ORBextractor*                extractor_;    //!< 把特征提取放到帧中
		};
	}
}

#endif // OPENSLAM_SLAM_FRAME_H_
