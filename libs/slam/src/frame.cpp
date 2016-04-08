#include <openslam/slam/frame.h>
#include <openslam/slam/feature.h>

namespace openslam
{
	namespace slam
	{
		long unsigned int Frame::frame_counter_ = 0;

		Frame::Frame(PinholeCamera* cam, const cv::Mat& img, double timestamp, ORBextractor* extractor) :
			id_(frame_counter_++),
			cam_(cam),
			img_(img),
			timestamp_(timestamp),
			extractor_(extractor)
		{
			// 获取尺度相关参数
			levels_num_ = extractor_->getLevels();
			scale_factor_ = extractor_->getScaleFactor();
			// 帧初始化的进行特征检测
			extractORB(img);
		}

		Frame::~Frame()
		{

		}

		void Frame::extractORB(const cv::Mat &image)
		{
			std::vector<cv::KeyPoint> vec_keypoints;
			cv::Mat descriptors;
			(*extractor_)(image, cv::Mat(), vec_keypoints, descriptors);
			keypoints_num_ = vec_keypoints.size();
			if (keypoints_num_ == 0) return;

			features_.reserve(keypoints_num_);
			for (size_t i = 0; i < vec_keypoints.size();i++)
			{
				Feature fea = Feature(this, vec_keypoints[i], descriptors.row(i));
				features_.push_back(fea);
			}
		}
	}
}

