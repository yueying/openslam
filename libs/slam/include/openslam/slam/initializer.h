/*************************************************************************
* 文件名： initializer.h
*
* 作者： 冯兵
* 邮件： fengbing123@gmail.com
* 时间： 2016/4/16
*
* 说明： 主要用来进行单目初始化
*************************************************************************/
#ifndef OPENSLAM_SLAM_INITIALIZER_H_
#define OPENSLAM_SLAM_INITIALIZER_H_

#include <openslam/slam/frame.h>

namespace openslam
{
	namespace slam
	{
		typedef std::pair<int, int> Match;
		/** \brief 主要用于单目slam中的初始化
		*/
		class SLAM_IMPEXP Initializer
		{
		public:

			/** \brief 传入sigma和iterations，用于判断计算初始位姿是采用
			*  单应矩阵还是基础矩阵，以及后期RANSAC的时候的最大迭代次数
			*/
			Initializer(float sigma = 1.0f, int iterations = 200);
			~Initializer();

			bool addFirstFrame(FramePtr ref_frame);
			bool addSecondFrame(FramePtr cur_frame);

			void reset();
		private:

			void findHomography(std::vector<bool> &matches_is_inliers, float &score, cv::Mat &H_cur_from_ref);
			void findFundamental(std::vector<bool> &is_inliers, float &score, cv::Mat &F21);

			/**
			* \brief 2D单应矩阵估计，点为归一化之后的点
			*/
			cv::Mat calcHFromMatches(const std::vector<cv::Point2f> &points_ref, const std::vector<cv::Point2f> &points_cur);
			/** \brief 基础矩阵的估计
			*/
			cv::Mat calcFFromMatches(const std::vector<cv::Point2f> &points_ref, const std::vector<cv::Point2f> &points_cur);

			float checkHomography(const cv::Mat &H_cur_from_ref, const cv::Mat &H_ref_from_cur, std::vector<bool> &matches_is_inliers, float sigma);
			float checkFundamental(const cv::Mat &F21, std::vector<bool> &matches_is_inliers, float sigma);

			bool reconstructF(std::vector<bool> &matches_is_inliers, cv::Mat &F21, cv::Mat &K,
				cv::Mat &R_cur_ref, cv::Mat &t_cur_ref, std::vector<cv::Point3f> &vec_point3d, std::vector<bool> &is_triangulated, float min_parallax, int min_triangulated);

			bool reconstructH(std::vector<bool> &matches_is_inliers, cv::Mat &H_cur_from_ref, cv::Mat &K,
				cv::Mat &R_cur_ref, cv::Mat &t_cur_ref, std::vector<cv::Point3f> &vec_point3d, std::vector<bool> &is_triangulated, float min_parallax, int min_triangulated);
			/** 三角定位，输入对应特征点，及对应投影矩阵计算特征点对应的三维点
			*/
			void triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

			void normalize(const std::vector<Feature *> &features, std::vector<cv::Point2f> &normalized_points, cv::Mat &T);

			int checkRt(const cv::Mat &R, const cv::Mat &t, const std::vector<Feature *> &vec_features1, const std::vector<Feature *> &vec_features2,
				const std::vector<Match> &matches_ref_cur, std::vector<bool> &is_inliers,
				const cv::Mat &K, std::vector<cv::Point3f> &vec_point3d, float th2, std::vector<bool> &vec_is_good, float &parallax);

			void decomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);

		public:
			FramePtr ref_frame_;//!<初始参考帧
			std::vector<int> init_matchex_;//!<初始的匹配
			std::vector<cv::Point3f> init_3d_points_;//!<初始化得到的3d点
			std::vector<Feature *> ref_features_;//!<参考帧中的特征
			std::vector<Feature *> cur_features_;//!<当前帧中的特征

		protected:
			float sigma_;//!< 标准差
			int max_iter_num_;//!<RANSAC 最大迭代次数
			
			std::vector<Match> matches_ref_cur_;//!<当前匹配，表示参考帧对当前帧
			std::vector<bool> is_matched_ref_;//!<用于表示参考帧是否已匹配
		
			std::vector<cv::Point2f> prev_matched_;//!<预先匹配点，也就是参考帧特征点的位置

			std::vector<std::vector<size_t> > ransac_sets_;//!<构建8点的随机数据集
		};
	}
}

#endif // OPENSLAM_SLAM_INITIALIZER_H_