#include <openslam/slam/map_point.h>
#include <openslam/slam/feature.h>
#include <openslam/slam/orb_matcher.h>

namespace openslam
{
	namespace slam
	{
		long unsigned int MapPoint::map_point_counter_ = 0;
		MapPoint::MapPoint(const cv::Mat &pos):
			id_(map_point_counter_++),
			world_position_(pos), 
			obs_num_(0)
		{
			normal_vector_ = cv::Mat::zeros(3, 1, CV_32F);
		}

		void MapPoint::addFeatureRef(Feature* ftr)
		{
			obs_.push_front(ftr);
			++obs_num_;
		}

		void MapPoint::computeDistinctiveDescriptors()
		{
			// 对同一个mappoint所有对应的所有特征的描述子进行处理，寻找最好描述子
			std::vector<cv::Mat> vec_descriptors;
			std::list<Feature*>   obs;
			{
				std::unique_lock<std::mutex> lock1(mutex_features_);
				obs = obs_;
			}
			if (obs.empty()) return;
			vec_descriptors.reserve(obs.size());

			for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
			{
				if (*it)
					vec_descriptors.push_back((*it)->descriptor_);
			}
			if (vec_descriptors.empty()) return;
			// 计算两两特征描述之间的距离
			const size_t N = vec_descriptors.size();
#ifdef _WIN32
			float **distances;
			distances = new float*[N];
			for (int i = 0; i < N; i++)
			{
				distances[i] = new float[N];
			}
#else
			float distances[N][N];
#endif

			for (size_t i = 0; i < N; i++)
			{
				distances[i][i] = 0;
				for (size_t j = i + 1; j < N; j++)
				{
					int distij = ORBmatcher::descriptorDistance(vec_descriptors[i], vec_descriptors[j]);
					distances[i][j] = distij;
					distances[j][i] = distij;
				}
			}

			// 对这些两两计算的描述子中寻找最小的中值
			int best_median = INT_MAX;
			int best_index = 0;
			for (size_t i = 0; i < N; i++)
			{
				// 对距离排序找到中值
				std::vector<int> vec_dists(distances[i], distances[i] + N);
				sort(vec_dists.begin(), vec_dists.end());
				int median = vec_dists[0.5*(N - 1)];

				if (median < best_median)
				{
					best_median = median;
					best_index = i;
				}
			}
#ifdef _WIN32
			for (int i = 0; i < N; i++)
				delete[]distances[i];
			delete[]distances;
#endif 

			{
				std::unique_lock<mutex> lock(mutex_features_);
				descriptor_ = vec_descriptors[best_index].clone();
			}
		}
	}
}