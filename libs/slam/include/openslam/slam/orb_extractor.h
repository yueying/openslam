#ifndef OPENSLAM_SLAM_ORB_EXTRACTOR_H_
#define OPENSLAM_SLAM_ORB_EXTRACTOR_H_

#include <openslam/slam/link_pragmas.h>
#include <vector>
#include <list>
#include <opencv/cv.h>


namespace openslam
{
	namespace slam
	{
		/* 一个图像区域的象限划分：:
		UL_(1)   |    UR_(0)
		----------|-----------
		BL_(2)   |    BR_(3)
		*/
		class  ExtractorNode
		{
		public:
			ExtractorNode() :is_no_more_(false){}

			/** \brief 划分四叉树
			*/
			void divideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

		public:
			std::vector<cv::KeyPoint> vec_keys_;
			cv::Point2i UL_, UR_, BL_, BR_;
			std::list<ExtractorNode>::iterator node_iter_;
			bool is_no_more_;
		};

		class SLAM_IMPEXP ORBextractor
		{
		public:

			ORBextractor(int features_num = 500, float scale_factor = 1.2f, int levels_num = 8,
				int default_fast_threshold = 20, int min_fast_threshold = 7);

			~ORBextractor(){}

			/** \brief 计算图像的orb特征及描述，将orb特征分配到一个四叉树当中
			*  目前mask参数是被忽略的，没有实现
			*/
			void operator()(cv::InputArray image, cv::InputArray mask,
				std::vector<cv::KeyPoint>& keypoints,
				cv::OutputArray descriptors);

			/** \brief 得到高斯金字塔的层数
			*/
			inline int getLevels()
			{
				return levels_num_;
			}
			/** \brief 得到金字塔图像之间的尺度参数
			*/
			inline float getScaleFactor()
			{
				return scale_factor_;
			}
			/** \brief 得到金字塔图像每层的尺度因子
			*/
			inline std::vector<float> getScaleFactors()
			{
				return vec_scale_factor_;
			}

		protected:
			/** \brief 计算图像金字塔
			*/
			void computePyramid(cv::Mat image);
			/** \brief 通过四叉树的方式计算特征点
			*/
			void computeKeyPointsQuadTree(std::vector<std::vector<cv::KeyPoint> >& all_keypoints);
			/** \brief 通过四叉树的方式分配特征点
			*/
			std::vector<cv::KeyPoint> distributeQuadTree(const std::vector<cv::KeyPoint>& vec_to_distribute_keys, const int &min_x,
				const int &max_x, const int &min_y, const int &max_y, const int &feature_num, const int &level);

			void computeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& all_keypoints);

		public:
			std::vector<cv::Mat> vec_image_pyramid_;//!<图像金字塔

		protected:
			std::vector<cv::Point> pattern_;//!<用于存放训练的模板

			int features_num_;//!<最多提取的特征点的数量
			int levels_num_;//!<高斯金字塔的层数
			float scale_factor_;//!<金字塔图像之间的尺度参数
			std::vector<float> vec_scale_factor_;//!<用于存储每层的尺度因子
			
			int default_fast_threshold_;//!<默认设置fast角点阈值20
			int min_fast_threshold_;//!<设置fast角点阈值为9

			std::vector<int> feature_num_per_level_;//!<每层特征的个数

			std::vector<int> umax_;//!< 用于存储计算特征方向时，图像每个v坐标对应最大的u坐标

			
		};
	}// namespace slam
} //namespace openslam


#endif // OPENSLAM_SLAM_ORB_EXTRACTOR_H_

