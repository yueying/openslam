#include <openslam/slam/keyframe.h>
#include <openslam/slam/feature.h>

namespace openslam
{
	namespace slam
	{
		long unsigned int KeyFrame::keyframe_counter_ = 0;
		KeyFrame::KeyFrame(const Frame &frame) :Frame(frame)
		{
			keyframe_id_ = keyframe_counter_++;
		}

		KeyFrame::~KeyFrame()
		{

		}

		float KeyFrame::computeSceneMedianDepth()
		{
			Features  features;
			cv::Mat Tcw;
			{
				std::unique_lock<mutex> lock(mutex_features_);
				std::unique_lock<mutex> lock2(mutex_pose_);
				features = features_;
				Tcw = Tcw_.clone();
			}

			std::vector<float> vec_depths;
			vec_depths.reserve(keypoints_num_);
			cv::Mat Rcw2 = Tcw.row(2).colRange(0, 3);
			Rcw2 = Rcw2.t();
			float zcw = Tcw.at<float>(2, 3);
			for (int i = 0; i < keypoints_num_; i++)
			{
				if (features[i])
				{
					MapPoint* map_point = features[i]->map_point_;
					cv::Mat x3Dw = map_point->getWorldPosition();
					float z = Rcw2.dot(x3Dw) + zcw;// 从世界坐标系转当前相机坐标系计算z
					vec_depths.push_back(z);
				}
			}
			std::sort(vec_depths.begin(), vec_depths.end());
			return vec_depths[(vec_depths.size() - 1) / 2];
		}
	}
}