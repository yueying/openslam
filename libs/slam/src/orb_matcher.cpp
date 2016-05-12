#include <openslam/slam/orb_matcher.h>
#include <openslam/slam/frame.h>
#include <openslam/slam/feature.h>
#include <openslam/slam/keyframe.h>

namespace openslam
{
	namespace slam
	{
		// 用于确定两个特征向量距离的最大阈值
		const int ORBmatcher::TH_HIGH = 100;
		// 用于确定两个特征向量距离的最小阈值
		const int ORBmatcher::TH_LOW = 50;
		const int ORBmatcher::HISTO_LENGTH = 30;

		ORBmatcher::ORBmatcher(float nnratio, bool check_orientation) :
			nn_ratio_(nnratio),
			is_check_orientation_(check_orientation)
		{

		}

		ORBmatcher::~ORBmatcher(){}

		int ORBmatcher::searchForInitialization(FramePtr ref_frame, FramePtr cur_frame,
			std::vector<cv::Point2f> &prev_matched, std::vector<int> &matches_ref_cur, int window_size)
		{
			int nmatches = 0;
			int keypoints_num = ref_frame->getKeypointsNum();
			matches_ref_cur = std::vector<int>(keypoints_num, -1);
			std::vector<int> rot_hist[HISTO_LENGTH];
			for (int i = 0; i < HISTO_LENGTH; i++)
				rot_hist[i].reserve(500);
			// 计算直方图的比例因子
			const float factor = 1.0f / HISTO_LENGTH;

			std::vector<int> matched_distance(keypoints_num, (std::numeric_limits<int>::max)());
			std::vector<int> matches_cur_ref(keypoints_num, -1);

			for (size_t i1 = 0, iend1 = keypoints_num; i1 < iend1; i1++)
			{
				cv::KeyPoint kp1 = ref_frame->features_[i1]->undistored_keypoint_;
				int level1 = kp1.octave;
				if (level1 > 0)//只考虑了原始图像
					continue;
				// 在当前帧中查找可能匹配的特征的索引
				std::vector<size_t> keypoint_indices = cur_frame->getFeaturesInArea(prev_matched[i1].x, prev_matched[i1].y, window_size, level1, level1);

				if (keypoint_indices.empty())
					continue;
				// 获取对应参考帧的特征描述
				cv::Mat cur_des = ref_frame->features_[i1]->descriptor_;

				int best_dist = (std::numeric_limits<int>::max)();
				int best_dist2 = (std::numeric_limits<int>::max)();
				int best_index2 = -1;
				// 对当前帧中可能的特征点进行遍历
				for (std::vector<size_t>::iterator vit = keypoint_indices.begin(); vit != keypoint_indices.end(); vit++)
				{
					size_t i2 = *vit;
					// 对应当前帧的特征描述
					cv::Mat ref_des = cur_frame->features_[i2]->descriptor_;

					int dist = descriptorDistance(cur_des, ref_des);

					if (matched_distance[i2] <= dist)
						continue;
					// 找到最小的前两个距离
					if (dist < best_dist)
					{
						best_dist2 = best_dist;
						best_dist = dist;
						best_index2 = i2;
					}
					else if (dist < best_dist2)
					{
						best_dist2 = dist;
					}
				}
				// 确保最小距离小于阈值
				if (best_dist <= TH_LOW)
				{
					// 再确保此最小距离乘以nn_ratio_要大于最小距离，主要确保该匹配比较鲁棒
					if (best_dist < (float)best_dist2*nn_ratio_)
					{
						// 如果已经匹配，则说明当前特征已经有过对应，则就会有两个对应，移除该匹配
						if (matches_cur_ref[best_index2] >= 0)
						{
							matches_ref_cur[matches_cur_ref[best_index2]] = -1;
							nmatches--;
						}
						// 记录匹配
						matches_ref_cur[i1] = best_index2;
						matches_cur_ref[best_index2] = i1;
						matched_distance[best_index2] = best_dist;
						nmatches++;

						if (is_check_orientation_)
						{
							float rot = ref_frame->features_[i1]->undistored_keypoint_.angle - cur_frame->features_[best_index2]->undistored_keypoint_.angle;
							if (rot < 0.0)
								rot += 360.0f;
							int bin = round(rot*factor);
							if (bin == HISTO_LENGTH)
								bin = 0;
							assert(bin >= 0 && bin < HISTO_LENGTH);
							rot_hist[bin].push_back(i1);//得到直方图
						}
					}
				}

			}

			if (is_check_orientation_)
			{
				int ind1 = -1;
				int ind2 = -1;
				int ind3 = -1;

				computeThreeMaxima(rot_hist, HISTO_LENGTH, ind1, ind2, ind3);

				for (int i = 0; i < HISTO_LENGTH; i++)
				{
					// 对可能的一致的方向就不予考虑
					if (i == ind1 || i == ind2 || i == ind3)
						continue;
					// 对剩下方向不一致的匹配进行剔除
					for (size_t j = 0, jend = rot_hist[i].size(); j < jend; j++)
					{
						int idx1 = rot_hist[i][j];
						if (matches_ref_cur[idx1] >= 0)
						{
							matches_ref_cur[idx1] = -1;
							nmatches--;
						}
					}
				}

			}

			//更新 prev matched
			for (size_t i1 = 0, iend1 = matches_ref_cur.size(); i1 < iend1; i1++)
				if (matches_ref_cur[i1] >= 0)
					prev_matched[i1] = cur_frame->features_[matches_ref_cur[i1]]->undistored_keypoint_.pt;

			return nmatches;

		}

		int ORBmatcher::searchByProjection(FramePtr cur_frame, const FramePtr last_frame, const float th)
		{
			int nmatches = 0;
			// 主要用于匹配特征方向判断
			std::vector<int> rot_hist[HISTO_LENGTH];
			for (int i = 0; i < HISTO_LENGTH; i++)
				rot_hist[i].reserve(500);
			const float factor = 1.0f / HISTO_LENGTH;

			// 开始对上一帧的所有map point进行遍历
			for (int i = 0; i < last_frame->getKeypointsNum(); i++)
			{
				MapPoint *map_point = last_frame->features_[i]->map_point_;
				if (map_point&&!map_point->is_outlier_)
				{
					// 判断当前帧是否能看到该map point
					cv::Mat x3Dw = map_point->getWorldPosition();
					cv::Point2f point2d;
					if (cur_frame->isInFrame(x3Dw, point2d))
						continue;
					int last_octave = last_frame->features_[i]->keypoint_.octave;//获得上一帧特征对应的尺度层数，在相同层进行匹配
					float radius = th*cur_frame->getScaleFactors()[last_octave];//确定搜索半径

					std::vector<size_t> keypoint_indices;//可能的匹配点
					keypoint_indices = cur_frame->getFeaturesInArea(point2d.x, point2d.y, radius, last_octave - 1, last_octave + 1);

					if (keypoint_indices.empty())//没找到，则返回
						continue;
					// 得到目前map point的最好的特征描述，用于匹配
					const cv::Mat map_point_desc = map_point->getDescriptor();
					int best_dist = 256;
					int best_index2 = -1;
					for (std::vector<size_t>::const_iterator vit = keypoint_indices.begin(), vend = keypoint_indices.end(); vit != vend; vit++)
					{
						const size_t i2 = *vit;
						Feature * feature = cur_frame->features_[i2];
						MapPoint *map_point = feature->map_point_;
						if (map_point)
						{
							if (map_point->observationsNum() > 0)//如果该map point已经有对应
								continue;
						}

						const cv::Mat &d = feature->descriptor_;

						const int dist = descriptorDistance(map_point_desc, d);

						if (dist < best_dist)
						{
							best_dist = dist;
							best_index2 = i2;
						}
					}

					if (best_dist <= TH_HIGH)
					{
						cur_frame->features_[best_index2]->map_point_ = map_point;
						nmatches++;

						if (is_check_orientation_)
						{
							float rot = last_frame->features_[i]->undistored_keypoint_.angle - cur_frame->features_[best_index2]->undistored_keypoint_.angle;
							if (rot < 0.0)
								rot += 360.0f;
							int bin = round(rot*factor);
							if (bin == HISTO_LENGTH)
								bin = 0;
							assert(bin >= 0 && bin < HISTO_LENGTH);
							rot_hist[bin].push_back(best_index2);
						}
					}

				}
			}

			if (is_check_orientation_)
			{
				int ind1 = -1;
				int ind2 = -1;
				int ind3 = -1;

				computeThreeMaxima(rot_hist, HISTO_LENGTH, ind1, ind2, ind3);

				for (int i = 0; i < HISTO_LENGTH; i++)
				{
					if (i != ind1 && i != ind2 && i != ind3)
					{
						for (size_t j = 0, jend = rot_hist[i].size(); j < jend; j++)
						{
							cur_frame->features_[rot_hist[i][j]]->map_point_ = static_cast<MapPoint*>(nullptr);
							nmatches--;
						}
					}
				}
			}

			return nmatches;


		}

		void ORBmatcher::computeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
		{
			int max1 = 0;
			int max2 = 0;
			int max3 = 0;

			for (int i = 0; i < L; i++)
			{
				const int s = histo[i].size();
				if (s > max1)
				{
					max3 = max2;
					max2 = max1;
					max1 = s;
					ind3 = ind2;
					ind2 = ind1;
					ind1 = i;
				}
				else if (s > max2)
				{
					max3 = max2;
					max2 = s;
					ind3 = ind2;
					ind2 = i;
				}
				else if (s > max3)
				{
					max3 = s;
					ind3 = i;
				}
			}
			// 如果此高度与最高度相差太大就不予考虑，设索引值为-1
			if (max2 < 0.1f*(float)max1)
			{
				ind2 = -1;
				ind3 = -1;
			}
			else if (max3 < 0.1f*(float)max1)
			{
				ind3 = -1;
			}
		}

		int ORBmatcher::descriptorDistance(const cv::Mat &a, const cv::Mat &b)
		{
			//TODO:???具体计算有待了解
			const int *pa = a.ptr<int32_t>();
			const int *pb = b.ptr<int32_t>();

			int dist = 0;

			for (int i = 0; i < 8; i++, pa++, pb++)
			{
				unsigned  int v = *pa ^ *pb;
				v = v - ((v >> 1) & 0x55555555);
				v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
				dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
			}

			return dist;
		}

		int ORBmatcher::searchByBoW(KeyFrame* keyframe, FramePtr frame)
		{
			// 关键帧上多有的特征对应的map point
			const std::vector<MapPoint*> map_points_in_keyframe = keyframe->getMapPointMatches();
			// 构建临时存放匹配的map point的点
			std::vector<MapPoint*> map_point_matches = std::vector<MapPoint*>(frame->getKeypointsNum(), static_cast<MapPoint*>(nullptr));

			const DBoW2::FeatureVector &feat_vector = keyframe->feat_vector_;

			int nmatches = 0;
			// 用于匹配方向判断
			std::vector<int> rot_hist[HISTO_LENGTH];
			for (int i = 0; i < HISTO_LENGTH; i++)
				rot_hist[i].reserve(500);
			const float factor = 1.0f / HISTO_LENGTH;
			// 对
			DBoW2::FeatureVector::const_iterator keyframe_feat_vector_it = feat_vector.begin();
			DBoW2::FeatureVector::const_iterator frame_feat_vector_it = frame->feat_vector_.begin();
			DBoW2::FeatureVector::const_iterator keyframe_feat_vector_end = feat_vector.end();
			DBoW2::FeatureVector::const_iterator frame_feat_vector_end = frame->feat_vector_.end();

			while (keyframe_feat_vector_it != keyframe_feat_vector_end && frame_feat_vector_it != frame_feat_vector_end)
			{
				if (keyframe_feat_vector_it->first == frame_feat_vector_it->first)
				{
					const std::vector<unsigned int> keyframe_indices = keyframe_feat_vector_it->second;
					const std::vector<unsigned int> frame_indices = frame_feat_vector_it->second;

					for (size_t keyframe_i = 0; keyframe_i < keyframe_indices.size(); keyframe_i++)
					{
						const unsigned int real_keyframe_id = keyframe_indices[keyframe_i];

						MapPoint* map_point = map_points_in_keyframe[real_keyframe_id];

						if (!map_point)
							continue;

						if (map_point->isBad())
							continue;

						const cv::Mat &keyframe_desc = keyframe->features_[real_keyframe_id]->descriptor_;

						int best_dist1 = 256;
						int best_frame_id = -1;
						int best_dist2 = 256;

						for (size_t frame_i = 0; frame_i < frame_indices.size(); frame_i++)
						{
							const unsigned int real_frame_id = frame_indices[frame_i];

							if (map_point_matches[real_frame_id])
								continue;

							const cv::Mat &frame_desc = frame->features_[real_frame_id]->descriptor_;

							const int dist = descriptorDistance(keyframe_desc, frame_desc);

							if (dist < best_dist1)
							{
								best_dist2 = best_dist1;
								best_dist1 = dist;
								best_frame_id = real_frame_id;
							}
							else if (dist < best_dist2)
							{
								best_dist2 = dist;
							}
						}

						if (best_dist1 <= TH_LOW)
						{
							if (static_cast<float>(best_dist1) < nn_ratio_*static_cast<float>(best_dist2))
							{
								map_point_matches[best_frame_id] = map_point;
								// 赋给对应的特征
								frame->features_[best_frame_id]->map_point_ = map_point;

								const cv::KeyPoint &kp = keyframe->features_[real_keyframe_id]->undistored_keypoint_;

								if (is_check_orientation_)
								{
									float rot = kp.angle - frame->features_[best_frame_id]->keypoint_.angle;
									if (rot < 0.0)
										rot += 360.0f;
									int bin = round(rot*factor);
									if (bin == HISTO_LENGTH)
										bin = 0;
									assert(bin >= 0 && bin < HISTO_LENGTH);
									rot_hist[bin].push_back(best_frame_id);
								}
								nmatches++;
							}
						}

					}

					keyframe_feat_vector_it++;
					frame_feat_vector_it++;
				}
				else if (keyframe_feat_vector_it->first < frame_feat_vector_it->first)
				{
					keyframe_feat_vector_it = feat_vector.lower_bound(frame_feat_vector_it->first);
				}
				else
				{
					frame_feat_vector_it = frame->feat_vector_.lower_bound(keyframe_feat_vector_it->first);
				}
			}


			if (is_check_orientation_)
			{
				int ind1 = -1;
				int ind2 = -1;
				int ind3 = -1;

				computeThreeMaxima(rot_hist, HISTO_LENGTH, ind1, ind2, ind3);

				for (int i = 0; i < HISTO_LENGTH; i++)
				{
					if (i == ind1 || i == ind2 || i == ind3)
						continue;
					for (size_t j = 0, jend = rot_hist[i].size(); j < jend; j++)
					{
						map_point_matches[rot_hist[i][j]] = static_cast<MapPoint*>(nullptr);
						nmatches--;
					}
				}
			}

			return nmatches;
		}
	}
}