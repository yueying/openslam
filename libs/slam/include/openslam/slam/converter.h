#ifndef OPENSLAM_SLAM_CONVERTER_H_
#define OPENSLAM_SLAM_CONVERTER_H_
#include <openslam/slam/link_pragmas.h>
#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
#include<g2o/types/types_six_dof_expmap.h>
#include<g2o/types/types_seven_dof_expmap.h>

namespace openslam
{
	namespace slam
	{
		/** \brief ÀàÐÍ×ª»»
		*/
		class SLAM_IMPEXP Converter
		{
		public:
			static g2o::SE3Quat toSE3Quat(const cv::Mat &cv_Rt);

			static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
			static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
			static cv::Mat toCvMat(const Eigen::Matrix<double, 4, 4> &m);
			static cv::Mat toCvMat(const Eigen::Matrix3d &m);
			static cv::Mat toCvMat(const Eigen::Matrix<double, 3, 1> &m);
			static cv::Mat toCvSE3(const Eigen::Matrix<double, 3, 3> &R, const Eigen::Matrix<double, 3, 1> &t);


			static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Mat &cv_vector);
			static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Point3f &cv_point);
			static Eigen::Matrix<double, 3, 3> toMatrix3d(const cv::Mat &cv_mat3);

			static std::vector<float> toQuaternion(const cv::Mat &M);
		};
	}
}

#endif // OPENSLAM_SLAM_CONVERTER_H_
