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
#include <memory>
#include <opencv2/opencv.hpp>
#include <DBoW2/BowVector.h>
#include <DBoW2/FeatureVector.h>

#include <openslam/slam/abstract_camera.h>
#include <openslam/slam/orb_extractor.h>
#include <openslam/slam/orb_vocabulary.h>

namespace openslam
{
	namespace slam
	{
		//目前通过宏设置为固定值，后期进行参数配置
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64
		class Feature;
		class KeyFrame;
		typedef std::vector<Feature *> Features;

		class SLAM_IMPEXP Frame
		{
		public:
			Frame(){}//用于转关键帧的时候

			/** 帧的构造，给出对应相机参数及对应原始帧及对应的时间码
			*/
			Frame(AbstractCamera* cam, const cv::Mat& img, double timestamp, ORBextractor* extractor, ORBVocabulary* voc = nullptr, bool is_rgb_order = false);
			virtual ~Frame();

			/** 往图像中添加特征
			*/
			void addFeature(Feature* ftr);

			/** \brief 将特征限制到某个区域内，更方便进行特征匹配
			*/
			std::vector<size_t> getFeaturesInArea(const float &x, const float  &y, const float  &r, const int min_level = -1, const int max_level = -1) const;

			/** 将描述子转成BoW的单词表示
			*/
			void computeBoW();

			/** 返回帧对应的相机的位置
			*/
			cv::Mat getCameraCenter();

			/** 用于得到金字塔图像的每层尺度值
			*/
			std::vector<float> getScaleFactors();
			/** 得到金字塔的层数
			*/
			inline int getLevelsNum(){ return levels_num_; }

			/** 得到关键点个数据
			*/
			inline int getKeypointsNum(){ return keypoints_num_; }

			/**得到相机外参*/
			inline cv::Mat getCameraExternal() { return Tcw_; }

			/**得到相机位姿*/
			inline cv::Mat getPose() { return Tcw_.inv(); }

			/**设置相机的外参矩阵*/
			inline void setCameraExternal(cv::Mat Tcw) { Tcw_ = Tcw.clone(); }

			/**判断三维点是否在图像帧内,通过看不考虑畸变的投影点是否在畸变处理之后的图像范围内*/
			bool isInFrame(const cv::Mat &point3d, cv::Point2f &point2d);
		protected:
			/**提供帧数据向关键帧数据的拷贝，不允许自身拷贝*/
			Frame(const Frame &frame);
			/**\brief图像进行预处理，将彩色图像转灰度
			*/
			void prepareImage(const cv::Mat& input_image, cv::Mat& gray_image);

			/** \brief 对图像进行ORB特征检测
			*/
			void extractORB(const cv::Mat &image);

			/** \brief 计算图像的边界，考虑图像畸变
			*/
			void computeImageBounds(const cv::Mat &image);

			/** \brief 每次图像帧构造的时候调用，用于将特征点分配到格子中以加快特征匹配。
			*/
			void assignFeaturesToGrid();

			/** 计算特征点所在单元格，不在格子内返回false
			*/
			bool posInGrid(const cv::KeyPoint &kp, int &pos_x, int &pos_y);

			/** \brief 将特征构建成vector形式，用于转换BoW结构
			*/
			std::vector<cv::Mat> toDescriptorVector(const Features &features);
		public:
			static long unsigned int     frame_counter_;//!< 创建帧的计数器，用于设置帧的唯一id
			long unsigned int            id_;           //!< 帧的id
			double                       timestamp_;    //!< 帧的时间戳
			AbstractCamera*              cam_;          //!< 相机模型
			cv::Mat                      img_;          //!< 帧对应的原始图像
			cv::Mat                      gray_img_;     //!< 帧对应的灰度图像
			bool                         is_rgb_order_; //!< 图像顺序，true图像是RGB的顺序，false是BGR的顺序
			float                        scale_factor_; //!< 对应金字塔图像的尺度因子		
			Features                     features_;     //!< 帧对应的特征
			cv::Mat                      Tcw_;        //!< 从世界坐标系(w)orld转到摄像机坐标系(c)amera，刚性变换Rt
			ORBextractor*                extractor_;    //!< 把特征提取放到帧中

			// Bag of Words Vector structures.
			ORBVocabulary* orb_vocabulary_;
			DBoW2::BowVector bow_vec_;
			DBoW2::FeatureVector feat_vector_;

			// 计算的图像边界主要考虑畸变图像			
			static float                 min_bound_x_;//!<畸变矫正后最小边界x的坐标
			static float                 max_bound_x_;//!<畸变矫正后最大边界x的坐标
			static float                 min_bound_y_;//!<畸变矫正后最小边界y的坐标
			static float                 max_bound_y_;//!<畸变矫正后最大边界y的坐标
			static float                 grid_element_height_;//!<划分之后格子的列数
			static float                 grid_element_width_;//!<划分之后格子的宽度
			static bool                  is_initial_computations_;//!<进行一次计算的标识，ture表示已经计算完成
			std::vector<std::size_t>     grid_[FRAME_GRID_COLS][FRAME_GRID_ROWS];//!<一幅图像分配的格子,里面存储特征点的索引

			KeyFrame*  ref_keyframe_;//!< 当前帧参考的关键帧
		protected:
			int                          keypoints_num_;//!< 特征点的个数
			int                          levels_num_;   //!< 对应金字塔的层数
		};

		typedef std::shared_ptr<Frame> FramePtr;
	}
}

#endif // OPENSLAM_SLAM_FRAME_H_
