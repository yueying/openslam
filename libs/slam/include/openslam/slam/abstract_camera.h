#ifndef OPENSLAM_SLAM_ABSTRACT_CAMERA_H_
#define OPENSLAM_SLAM_ABSTRACT_CAMERA_H_
#include <openslam/slam/link_pragmas.h>
#include <opencv2/core/core.hpp>

namespace openslam
{
	namespace slam
	{
		class SLAM_IMPEXP AbstractCamera {

		public:
			// 考虑畸变参数k1,k2,p1,p2,k3
			AbstractCamera(double width, double height,
				double fx, double fy, double cx, double cy,
				double k1 = 0.0, double k2 = 0.0, double p1 = 0.0, double p2 = 0.0, double k3 = 0.0);

			virtual ~AbstractCamera() {}

			/// 返回x方向的焦距值
			virtual double getFocalLength() const
			{
				return fabs(fx_);
			}

			/**针对RGB-D深度图里给的数据与实际距离的比例因子，预留接口*/
			virtual double getMapFactor() const = 0;

			/**针对双目相机预留接口*/
			virtual double getBaselineMultFx() const = 0;

			/// 获得矫正之后的图像，主要用于显示
			void undistortImage(const cv::Mat& raw, cv::Mat& rectified);

			/// 返回相机分辨率的宽度
			inline int width() const { return width_; }
			/// 返回相机分辨率的高度
			inline int height() const { return height_; }
			/// 分别得到相机矩阵的4个参数
			inline double fx() const { return fx_; }
			inline double fy() const { return fy_; }
			inline double cx() const { return cx_; }
			inline double cy() const { return cy_; }

			inline double invfx() const
			{
				if (fx_ == 0) return 0;
				return 1.0 / fx_;
			}

			inline double invfy() const
			{
				if (fy_ == 0) return 0;
				return 1.0 / fy_;
			}

			inline cv::Mat cvK() const { return cvK_; }

			inline cv::Mat distCoef() const{ return dist_coef_; }

		private:
			int width_; //!< 相机分辨率的宽度
			int height_; //!< 相机分辨率的高度
			double fx_, fy_;  //!< 相机两个方向的焦距值
			double cx_, cy_;  //!< 相机的中心点
			bool distortion_; //!< 是单纯的小孔相机模型，还是带有畸变？
			double d_[5];     //!< 畸变参数，参考 http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
			cv::Mat cvK_, dist_coef_;//!< 通过OpenCV表示的相机的相机矩阵和相机畸变参数
			cv::Mat undist_map1_, undist_map2_;//!<相机畸变在两个方向的map，提供给remap函数使用
		};
	}
}

#endif // OPENSLAM_SLAM_ABSTRACT_CAMERA_H_
