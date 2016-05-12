#include <openslam/slam/converter.h>

namespace openslam
{
	namespace slam
	{
		g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cv_Rt)
		{
			Eigen::Matrix3d R;
			R << cv_Rt.at<float>(0, 0), cv_Rt.at<float>(0, 1), cv_Rt.at<float>(0, 2),
				cv_Rt.at<float>(1, 0), cv_Rt.at<float>(1, 1), cv_Rt.at<float>(1, 2),
				cv_Rt.at<float>(2, 0), cv_Rt.at<float>(2, 1), cv_Rt.at<float>(2, 2);

			Eigen::Vector3d t(cv_Rt.at<float>(0, 3), cv_Rt.at<float>(1, 3), cv_Rt.at<float>(2, 3));

			return g2o::SE3Quat(R, t);
		}

		cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3)
		{
			Eigen::Matrix<double, 4, 4> eigMat = SE3.to_homogeneous_matrix();
			return toCvMat(eigMat);
		}

		cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3)
		{
			Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
			Eigen::Vector3d eigt = Sim3.translation();
			double s = Sim3.scale();
			return toCvSE3(s*eigR, eigt);
		}

		cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 4, 4> &m)
		{
			cv::Mat cvMat(4, 4, CV_32F);
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
					cvMat.at<float>(i, j) = m(i, j);

			return cvMat.clone();
		}

		cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
		{
			cv::Mat cvMat(3, 3, CV_32F);
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					cvMat.at<float>(i, j) = m(i, j);

			return cvMat.clone();
		}

		cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 3, 1> &m)
		{
			cv::Mat cvMat(3, 1, CV_32F);
			for (int i = 0; i < 3; i++)
				cvMat.at<float>(i) = m(i);

			return cvMat.clone();
		}

		cv::Mat Converter::toCvSE3(const Eigen::Matrix<double, 3, 3> &R, const Eigen::Matrix<double, 3, 1> &t)
		{
			cv::Mat cvMat = cv::Mat::eye(4, 4, CV_32F);
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					cvMat.at<float>(i, j) = R(i, j);
				}
			}
			for (int i = 0; i < 3; i++)
			{
				cvMat.at<float>(i, 3) = t(i);
			}

			return cvMat.clone();
		}

		Eigen::Matrix<double, 3, 1> Converter::toVector3d(const cv::Mat &cv_vector)
		{
			Eigen::Matrix<double, 3, 1> v;
			v << cv_vector.at<float>(0), cv_vector.at<float>(1), cv_vector.at<float>(2);

			return v;
		}

		Eigen::Matrix<double, 3, 1> Converter::toVector3d(const cv::Point3f &cv_point)
		{
			Eigen::Matrix<double, 3, 1> v;
			v << cv_point.x, cv_point.y, cv_point.z;

			return v;
		}

		Eigen::Matrix<double, 3, 3> Converter::toMatrix3d(const cv::Mat &cv_mat3)
		{
			Eigen::Matrix<double, 3, 3> M;

			M << cv_mat3.at<float>(0, 0), cv_mat3.at<float>(0, 1), cv_mat3.at<float>(0, 2),
				cv_mat3.at<float>(1, 0), cv_mat3.at<float>(1, 1), cv_mat3.at<float>(1, 2),
				cv_mat3.at<float>(2, 0), cv_mat3.at<float>(2, 1), cv_mat3.at<float>(2, 2);

			return M;
		}

		std::vector<float> Converter::toQuaternion(const cv::Mat &M)
		{
			Eigen::Matrix<double, 3, 3> eigMat = toMatrix3d(M);
			Eigen::Quaterniond q(eigMat);

			std::vector<float> v(4);
			v[0] = q.x();
			v[1] = q.y();
			v[2] = q.z();
			v[3] = q.w();

			return v;
		}

	}
}