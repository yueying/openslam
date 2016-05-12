#include <openslam/slam/keyframe.h>
#include <openslam/slam/feature.h>

namespace openslam
{
	namespace slam
	{
		long unsigned int KeyFrame::keyframe_counter_ = 0;
		KeyFrame::KeyFrame(const Frame &frame) :Frame(frame), is_bad_(false),
			gba_for_keyframe_num_(0),
			local_ba_for_keyframe_id_(0),
			fixed_ba_for_keyframe_id_(0)
		{
			//保存自己的index索引
			keyframe_id_ = keyframe_counter_++;
			//将对应特征的帧的指针引用该为自己
			std::for_each(features_.begin(), features_.end(), [&](Feature* ftr){ if (ftr) ftr->addFrameRef(this); });

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

		int KeyFrame::trackedMapPoints(const int &min_obs)
		{
			std::unique_lock<mutex> lock(mutex_features_);

			int points_num = 0;
			const bool check_obs = min_obs > 0;
			for (int i = 0; i < keypoints_num_; i++)
			{
				MapPoint* map_point = features_[i]->map_point_;
				if (map_point)
				{
					if (!map_point->isBad())
					{
						if (check_obs)
						{
							if (map_point->observationsNum() >= min_obs)
								points_num++;
						}
						else
							points_num++;
					}
				}
			}

			return points_num;
		}

		void KeyFrame::setBadFlag()
		{
			std::unique_lock<mutex> lock(mutex_connections_);
			is_bad_ = true;
		}

		bool KeyFrame::isBad()
		{
			std::unique_lock<mutex> lock(mutex_connections_);
			return is_bad_;
		}

		std::vector<MapPoint*> KeyFrame::getMapPointMatches()
		{
			std::unique_lock<mutex> lock(mutex_features_);
			std::vector<MapPoint*>  vec_map_points;
			for (int i = 0; i < keypoints_num_; i++)
			{
				MapPoint* map_point = features_[i]->map_point_;
				vec_map_points.push_back(map_point);
			}
			return vec_map_points;
		}
	}
}