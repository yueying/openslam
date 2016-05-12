#include <openslam/slam/optimizer.h>

#include <mutex>
#include <Eigen/StdVector>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/linear_solver_eigen.h>
#include <g2o/types/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/linear_solver_dense.h>
#include <g2o/types/types_seven_dof_expmap.h>

#include <openslam/slam/converter.h>
#include <openslam/slam/feature.h>

namespace openslam
{
	namespace slam
	{
		void Optimizer::bundleAdjustment(const std::vector<KeyFrame *> &vec_keyframes, const std::vector<MapPoint *> &vec_map_points,
			int iter_num, bool* stop_flag, const unsigned long loop_keyframe_num, const bool is_robust)
		{
			std::vector<bool> is_not_include_mappoint;
			is_not_include_mappoint.resize(vec_map_points.size());
			// 初始化优化方法
			g2o::SparseOptimizer optimizer;
			g2o::BlockSolver_6_3::LinearSolverType * linear_solver;
			// 初始化求解器
			linear_solver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
			// 设置优化算法
			g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linear_solver);
			g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
			optimizer.setAlgorithm(solver);

			// 设置是否强制终止，需要终止迭代的时候强制终止
			if (stop_flag)
				optimizer.setForceStopFlag(stop_flag);

			long unsigned int max_keyframe_id = 0;

			// 向优化中添加关键帧顶点
			for (size_t i = 0; i < vec_keyframes.size(); i++)
			{
				KeyFrame* keyframe = vec_keyframes[i];
				if (keyframe->isBad())
					continue;
				g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
				vSE3->setEstimate(Converter::toSE3Quat(keyframe->getCameraExternal()));
				vSE3->setId(keyframe->keyframe_id_);
				vSE3->setFixed(keyframe->keyframe_id_ == 0);//设置第一个点固定不用优化
				optimizer.addVertex(vSE3);
				if (keyframe->keyframe_id_ > max_keyframe_id)//记录当前关键帧所在帧的最大id
					max_keyframe_id = keyframe->keyframe_id_;
			}

			const float thHuber2D = sqrt(5.99);
			const float thHuber3D = sqrt(7.815);

			// 向优化中添加map point顶点
			for (size_t i = 0; i<vec_map_points.size(); i++)
			{
				MapPoint* map_point = vec_map_points[i];
				if (map_point->isBad())
					continue;
				g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
				vPoint->setEstimate(Converter::toVector3d(map_point->getWorldPosition()));
				const int id = map_point->id_ + max_keyframe_id + 1;
				vPoint->setId(id);
				vPoint->setMarginalized(true);
				optimizer.addVertex(vPoint);
				// 得到map point观测到的所有对应特征
				const std::list<Feature*> observations = map_point->getObservations();

				int edges_num = 0;

				for (std::list<Feature*>::const_iterator mit = observations.begin(); mit != observations.end(); mit++)
				{
					//map point对应的特征所在的帧肯定是关键帧
					KeyFrame *keyframe = dynamic_cast<KeyFrame *>((*mit)->frame_);
					if (keyframe->isBad() || keyframe->keyframe_id_>max_keyframe_id)
						continue;

					edges_num++;

					const cv::KeyPoint &kpUn = (*mit)->undistored_keypoint_;

					Eigen::Matrix<double, 2, 1> obs;
					obs << kpUn.pt.x, kpUn.pt.y;
					// 设置边，连接两个顶点的边
					g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

					e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
					e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(keyframe->keyframe_id_)));
					e->setMeasurement(obs);
					const float sigma = keyframe->getScaleFactors()[kpUn.octave];
					const float &inv_sigma2 = 1.0f / (sigma*sigma);
					e->setInformation(Eigen::Matrix2d::Identity()*inv_sigma2);

					if (is_robust)
					{
						g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(thHuber2D);
					}
					AbstractCamera *cam = keyframe->cam_;
					e->fx = cam->fx();
					e->fy = cam->fy();
					e->cx = cam->cx();
					e->cy = cam->cy();
					// 在优化中添加边
					optimizer.addEdge(e);


				}

				if (edges_num == 0)
				{
					optimizer.removeVertex(vPoint);
					is_not_include_mappoint[i] = true;
				}
				else
				{
					is_not_include_mappoint[i] = false;
				}
			}

			// 进行优化
			optimizer.initializeOptimization();
			// 设置优化迭代次数
			optimizer.optimize(iter_num);

			// 优化完成之后恢复数据，得到优化关键帧的值
			for (size_t i = 0; i < vec_keyframes.size(); i++)
			{
				KeyFrame* keyframe = vec_keyframes[i];
				if (keyframe->isBad())
					continue;
				g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(keyframe->keyframe_id_));
				g2o::SE3Quat SE3quat = vSE3->estimate();
				if (loop_keyframe_num == 0)
				{
					keyframe->setCameraExternal(Converter::toCvMat(SE3quat));
				}
				else
				{
					keyframe->gba_Tcw_.create(4, 4, CV_32F);
					Converter::toCvMat(SE3quat).copyTo(keyframe->gba_Tcw_);
					keyframe->gba_for_keyframe_num_ = loop_keyframe_num;
				}
			}

			//得到优化map point的值
			for (size_t i = 0; i < vec_map_points.size(); i++)
			{
				if (is_not_include_mappoint[i])
					continue;

				MapPoint* map_point = vec_map_points[i];

				if (map_point->isBad())
					continue;
				g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(map_point->id_ + max_keyframe_id + 1));

				if (loop_keyframe_num == 0)
				{
					map_point->setWorldPosition(Converter::toCvMat(vPoint->estimate()));
					map_point->updateNormalAndDepth();
				}
				else
				{
					map_point->gba_pos_.create(3, 1, CV_32F);
					Converter::toCvMat(vPoint->estimate()).copyTo(map_point->gba_pos_);
					map_point->gba_for_keyframe_num_ = loop_keyframe_num;
				}
			}

		}

		void Optimizer::globalBundleAdjustemnt(Map* map, int iter_num, bool* stop_flag, const unsigned long loop_keyframe_num, const bool is_robust)
		{
			std::vector<KeyFrame*> vec_keyframes = map->getAllKeyFrames();
			std::vector<MapPoint*> vec_map_points = map->getAllMapPoints();
			bundleAdjustment(vec_keyframes, vec_map_points, iter_num, stop_flag, loop_keyframe_num, is_robust);
		}

		int Optimizer::poseOptimization(FramePtr frame)
		{
			// 初始化优化方法
			g2o::SparseOptimizer optimizer;
			g2o::BlockSolver_6_3::LinearSolverType * linear_solver;
			// 初始化求解器
			linear_solver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

			g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linear_solver);
			// 设置优化算法
			g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
			optimizer.setAlgorithm(solver);

			int initial_correspondences_num = 0;

			// 向优化中添加帧为顶点
			g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
			vSE3->setEstimate(Converter::toSE3Quat(frame->Tcw_));
			vSE3->setId(0);
			vSE3->setFixed(false);
			optimizer.addVertex(vSE3);

			// 添加map point顶点
			const int keypoints_num = frame->getKeypointsNum();

			std::vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vec_edges_mono;
			std::vector<size_t> vnIndexEdgeMono;
			vec_edges_mono.reserve(keypoints_num);
			vnIndexEdgeMono.reserve(keypoints_num);

			const float delta_mono = sqrt(5.991);
			const float delta_stereo = sqrt(7.815);

			{
				std::unique_lock<mutex> lock(MapPoint::global_mutex_);

				for (int i = 0; i < keypoints_num; i++)
				{
					MapPoint* map_point = frame->features_[i]->map_point_;
					if (map_point)
					{
						initial_correspondences_num++;
						map_point->is_outlier_ = false;

						Eigen::Matrix<double, 2, 1> obs;
						const cv::KeyPoint &undistored_keypoint = frame->features_[i]->undistored_keypoint_;
						obs << undistored_keypoint.pt.x, undistored_keypoint.pt.y;

						g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
						e->setMeasurement(obs);
						const float sigma = frame->getScaleFactors()[undistored_keypoint.octave];
						const float inv_sigma2 = 1.0f / (sigma*sigma);
						e->setInformation(Eigen::Matrix2d::Identity()*inv_sigma2);

						g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(delta_mono);

						AbstractCamera *cam = frame->cam_;
						e->fx = cam->fx();
						e->fy = cam->fy();
						e->cx = cam->cx();
						e->cy = cam->cy();
						cv::Mat Xw = map_point->getWorldPosition();
						e->Xw[0] = Xw.at<float>(0);
						e->Xw[1] = Xw.at<float>(1);
						e->Xw[2] = Xw.at<float>(2);

						optimizer.addEdge(e);

						vec_edges_mono.push_back(e);
						vnIndexEdgeMono.push_back(i);


					}

				}
			}


			if (initial_correspondences_num < 3)
				return 0;

			// We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
			// At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
			const float chi2_mono[4] = { 5.991, 5.991, 5.991, 5.991 };
			const int its[4] = { 10, 10, 10, 10 };

			int bad_num = 0;
			for (size_t it = 0; it < 4; it++)
			{

				vSE3->setEstimate(Converter::toSE3Quat(frame->Tcw_));
				optimizer.initializeOptimization(0);
				optimizer.optimize(its[it]);

				bad_num = 0;
				for (size_t i = 0, iend = vec_edges_mono.size(); i < iend; i++)
				{
					g2o::EdgeSE3ProjectXYZOnlyPose* e = vec_edges_mono[i];

					const size_t idx = vnIndexEdgeMono[i];

					bool is_outlier = frame->features_[idx]->map_point_->is_outlier_;
					if (is_outlier)
					{
						e->computeError();
					}

					const float chi2 = e->chi2();

					if (chi2 > chi2_mono[it])
					{
						is_outlier = true;
						e->setLevel(1);
						bad_num++;
					}
					else
					{
						is_outlier = false;
						e->setLevel(0);
					}

					if (it == 2)
						e->setRobustKernel(0);
				}

				if (optimizer.edges().size() < 10)
					break;
			}

			// Recover optimized pose and return number of inliers
			g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
			g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
			cv::Mat pose = Converter::toCvMat(SE3quat_recov);
			frame->setCameraExternal(pose);

			return initial_correspondences_num - bad_num;
		}

		void Optimizer::localBundleAdjustment(KeyFrame *keyframe, bool* stop_flag, Map* map)
		{
			// Local KeyFrames: First Breath Search from Current Keyframe
			std::list<KeyFrame*> list_local_keyframes;

			list_local_keyframes.push_back(keyframe);
			keyframe->local_ba_for_keyframe_id_ = keyframe->id_;

			std::vector<KeyFrame*> vec_neigh_keyframes;
			map->getVectorCovisibleKeyFrames(keyframe, vec_neigh_keyframes);
			for (int i = 0, iend = vec_neigh_keyframes.size(); i < iend; i++)
			{
				KeyFrame* keyframe = vec_neigh_keyframes[i];
				keyframe->local_ba_for_keyframe_id_ = keyframe->id_;
				if (!keyframe->isBad())
					list_local_keyframes.push_back(keyframe);
			}

			// Local MapPoints seen in Local KeyFrames
			std::list<MapPoint*> list_local_mappoints;
			for (std::list<KeyFrame*>::iterator lit = list_local_keyframes.begin(), lend = list_local_keyframes.end(); lit != lend; lit++)
			{
				std::vector<MapPoint*> vec_map_points = (*lit)->getMapPointMatches();
				for (std::vector<MapPoint*>::iterator vit = vec_map_points.begin(), vend = vec_map_points.end(); vit != vend; vit++)
				{
					MapPoint* map_point = *vit;
					if (map_point)
						if (!map_point->isBad())
							if (map_point->local_ba_for_keyframe_id_ != keyframe->id_)
							{
								list_local_mappoints.push_back(map_point);
								map_point->local_ba_for_keyframe_id_ = keyframe->id_;
							}
				}
			}

			// Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
			std::list<KeyFrame*> lFixedCameras;
			for (std::list<MapPoint*>::iterator lit = list_local_mappoints.begin(), lend = list_local_mappoints.end(); lit != lend; lit++)
			{
				std::list<Feature*> observations = (*lit)->getObservations();
				for (std::list<Feature*>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
				{
					KeyFrame* keyframe = dynamic_cast<KeyFrame *>((*mit)->frame_);

					if (keyframe->local_ba_for_keyframe_id_ != keyframe->id_ && keyframe->fixed_ba_for_keyframe_id_ != keyframe->id_)
					{
						keyframe->fixed_ba_for_keyframe_id_ = keyframe->id_;
						if (!keyframe->isBad())
							lFixedCameras.push_back(keyframe);
					}
				}
			}

			// 初始化优化
			g2o::SparseOptimizer optimizer;
			g2o::BlockSolver_6_3::LinearSolverType * linear_solver;

			linear_solver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

			g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linear_solver);
			// 设置优化算法
			g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
			optimizer.setAlgorithm(solver);

			if (stop_flag)
				optimizer.setForceStopFlag(stop_flag);

			unsigned long max_keyframe_id = 0;

			// Set Local KeyFrame vertices
			for (list<KeyFrame*>::iterator lit = list_local_keyframes.begin(), lend = list_local_keyframes.end(); lit != lend; lit++)
			{
				KeyFrame* keyframe = *lit;
				g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
				vSE3->setEstimate(Converter::toSE3Quat(keyframe->getPose()));
				vSE3->setId(keyframe->id_);
				vSE3->setFixed(keyframe->id_ == 0);
				optimizer.addVertex(vSE3);
				if (keyframe->id_ > max_keyframe_id)
					max_keyframe_id = keyframe->id_;
			}

			// Set Fixed KeyFrame vertices
			for (std::list<KeyFrame*>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++)
			{
				KeyFrame* keyframe = *lit;
				g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
				vSE3->setEstimate(Converter::toSE3Quat(keyframe->getPose()));
				vSE3->setId(keyframe->id_);
				vSE3->setFixed(true);
				optimizer.addVertex(vSE3);
				if (keyframe->id_ > max_keyframe_id)
					max_keyframe_id = keyframe->id_;
			}

			// Set MapPoint vertices
			const int nExpectedSize = (list_local_keyframes.size() + lFixedCameras.size())*list_local_mappoints.size();

			std::vector<g2o::EdgeSE3ProjectXYZ*> vec_edges_mono;
			vec_edges_mono.reserve(nExpectedSize);

			std::vector<KeyFrame*> vpEdgeKFMono;
			vpEdgeKFMono.reserve(nExpectedSize);

			std::vector<MapPoint*> vpMapPointEdgeMono;
			vpMapPointEdgeMono.reserve(nExpectedSize);

			std::vector<g2o::EdgeStereoSE3ProjectXYZ*> vec_edges_stereo;
			vec_edges_stereo.reserve(nExpectedSize);

			std::vector<KeyFrame*> vpEdgeKFStereo;
			vpEdgeKFStereo.reserve(nExpectedSize);

			std::vector<MapPoint*> vpMapPointEdgeStereo;
			vpMapPointEdgeStereo.reserve(nExpectedSize);

			const float thHuberMono = sqrt(5.991);
			const float thHuberStereo = sqrt(7.815);

			for (std::list<MapPoint*>::iterator lit = list_local_mappoints.begin(), lend = list_local_mappoints.end(); lit != lend; lit++)
			{
				MapPoint* map_point = *lit;
				g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
				vPoint->setEstimate(Converter::toVector3d(map_point->getWorldPosition()));
				int id = map_point->id_ + max_keyframe_id + 1;
				vPoint->setId(id);
				vPoint->setMarginalized(true);
				optimizer.addVertex(vPoint);

				const std::list<Feature *> observations = map_point->getObservations();

				//Set edges
				for (std::list<Feature *>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
				{
					KeyFrame* keyframe = dynamic_cast<KeyFrame*>((*mit)->frame_);

					if (!keyframe->isBad())
					{
						const cv::KeyPoint &kpUn = (*mit)->undistored_keypoint_;

						Eigen::Matrix<double, 2, 1> obs;
						obs << kpUn.pt.x, kpUn.pt.y;

						g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

						e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
						e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(keyframe->id_)));
						e->setMeasurement(obs);
						const float sigma = keyframe->getScaleFactors()[kpUn.octave];
						const float &inv_sigma2 = 1.0f / (sigma*sigma);
						e->setInformation(Eigen::Matrix2d::Identity()*inv_sigma2);

						g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
						e->setRobustKernel(rk);
						rk->setDelta(thHuberMono);

						AbstractCamera *cam = keyframe->cam_;
						e->fx = cam->fx();
						e->fy = cam->fy();
						e->cx = cam->cx();
						e->cy = cam->cy();

						optimizer.addEdge(e);
						vec_edges_mono.push_back(e);
						vpEdgeKFMono.push_back(keyframe);
						vpMapPointEdgeMono.push_back(map_point);

					}
				}
			}

			if (stop_flag)
				if (*stop_flag)
					return;

			optimizer.initializeOptimization();
			optimizer.optimize(5);

			bool bDoMore = true;

			if (stop_flag)
				if (*stop_flag)
					bDoMore = false;

			if (bDoMore)
			{

				// Check inlier observations
				for (size_t i = 0, iend = vec_edges_mono.size(); i < iend; i++)
				{
					g2o::EdgeSE3ProjectXYZ* e = vec_edges_mono[i];
					MapPoint* map_point = vpMapPointEdgeMono[i];

					if (map_point->isBad())
						continue;

					if (e->chi2() > 5.991 || !e->isDepthPositive())
					{
						e->setLevel(1);
					}

					e->setRobustKernel(0);
				}

				for (size_t i = 0, iend = vec_edges_stereo.size(); i < iend; i++)
				{
					g2o::EdgeStereoSE3ProjectXYZ* e = vec_edges_stereo[i];
					MapPoint* map_point = vpMapPointEdgeStereo[i];

					if (map_point->isBad())
						continue;

					if (e->chi2() > 7.815 || !e->isDepthPositive())
					{
						e->setLevel(1);
					}

					e->setRobustKernel(0);
				}

				// Optimize again without the outliers

				optimizer.initializeOptimization(0);
				optimizer.optimize(10);

			}

			std::vector<pair<KeyFrame*, MapPoint*> > vToErase;
			vToErase.reserve(vec_edges_mono.size() + vec_edges_stereo.size());

			// Check inlier observations       
			for (size_t i = 0, iend = vec_edges_mono.size(); i < iend; i++)
			{
				g2o::EdgeSE3ProjectXYZ* e = vec_edges_mono[i];
				MapPoint* map_point = vpMapPointEdgeMono[i];

				if (map_point->isBad())
					continue;

				if (e->chi2() > 5.991 || !e->isDepthPositive())
				{
					KeyFrame* keyframe = vpEdgeKFMono[i];
					vToErase.push_back(std::make_pair(keyframe, map_point));
				}
			}

			for (size_t i = 0, iend = vec_edges_stereo.size(); i < iend; i++)
			{
				g2o::EdgeStereoSE3ProjectXYZ* e = vec_edges_stereo[i];
				MapPoint* map_point = vpMapPointEdgeStereo[i];

				if (map_point->isBad())
					continue;

				if (e->chi2() > 7.815 || !e->isDepthPositive())
				{
					KeyFrame* keyframe = vpEdgeKFStereo[i];
					vToErase.push_back(std::make_pair(keyframe, map_point));
				}
			}

			// Get Map Mutex
			std::unique_lock<mutex> lock(map->mutex_map_update_);

			if (!vToErase.empty())
			{
				for (size_t i = 0; i < vToErase.size(); i++)
				{
					KeyFrame* keyframe = vToErase[i].first;
					MapPoint* pMPi = vToErase[i].second;
					//TODO：这边的map point的删除
					//keyframe->eraseMapPointMatch(pMPi);
					//pMPi->eraseObservation(keyframe);
				}
			}

			// Recover optimized data

			//Keyframes
			for (std::list<KeyFrame*>::iterator lit = list_local_keyframes.begin(), lend = list_local_keyframes.end(); lit != lend; lit++)
			{
				KeyFrame* keyframe = *lit;
				g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(keyframe->id_));
				g2o::SE3Quat SE3quat = vSE3->estimate();
				keyframe->setCameraExternal(Converter::toCvMat(SE3quat));
			}

			//Points
			for (std::list<MapPoint*>::iterator lit = list_local_mappoints.begin(), lend = list_local_mappoints.end(); lit != lend; lit++)
			{
				MapPoint* map_point = *lit;
				g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(map_point->id_ + max_keyframe_id + 1));
				map_point->setWorldPosition(Converter::toCvMat(vPoint->estimate()));
				map_point->updateNormalAndDepth();
			}
		}



	}
}
