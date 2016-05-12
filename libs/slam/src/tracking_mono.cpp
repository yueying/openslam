#include <openslam/slam/tracking_mono.h>
#include <openslam/utils/notify.h>
#include <openslam/slam/feature.h>

namespace openslam
{
	namespace slam
	{
		TrackingMono::TrackingMono(const std::string &setting_path_name, ORBVocabulary *voc):
			TrackingBase(voc)
		{
			// 从配置文件中导入相机内参
			cv::FileStorage file_settings(setting_path_name, cv::FileStorage::READ);
			// 在什么大小的图片下标定这个是要知道的
			float width = file_settings["Camera.width"];
			float height = file_settings["Camera.height"];
			float fx = file_settings["Camera.fx"];
			float fy = file_settings["Camera.fy"];
			float cx = file_settings["Camera.cx"];
			float cy = file_settings["Camera.cy"];
			
			// 得到相机畸变参数
			float k1 = file_settings["Camera.k1"];
			float k2 = file_settings["Camera.k2"];
			float p1 = file_settings["Camera.p1"];
			float p2 = file_settings["Camera.p2"];
			float k3 = file_settings["Camera.k3"];
			cam_ = new MonocularCamera(width, height, fx, fy, cx, cy, k1, k2, p1, p2, k3);

			int is_rgb = file_settings["Camera.RGB"];
			is_rgb_order_ = is_rgb;

			if (is_rgb_order_)
				OPENSLAM_INFO << "- color order: RGB (ignored if grayscale)" << std::endl;
			else
				OPENSLAM_INFO << "- color order: BGR (ignored if grayscale)" << std::endl;

			// 导入ORB的相关参数
			int feature_num = file_settings["ORBextractor.nFeatures"];
			float scale_factor = file_settings["ORBextractor.scaleFactor"];
			int levels_num = file_settings["ORBextractor.nLevels"];
			int default_fast_threshold = file_settings["ORBextractor.iniThFAST"];
			int min_fast_threshold = file_settings["ORBextractor.minThFAST"];

			extractor_ = new ORBextractor(feature_num, scale_factor, levels_num, default_fast_threshold, min_fast_threshold);

			init_orb_extractor_ = new ORBextractor(2 * feature_num, scale_factor, levels_num, default_fast_threshold, min_fast_threshold);

			OPENSLAM_INFO << std::endl << "ORB Extractor Parameters: " << std::endl;
			OPENSLAM_INFO << "- Number of Features: " << feature_num << std::endl;
			OPENSLAM_INFO << "- Scale Levels: " << levels_num << std::endl;
			OPENSLAM_INFO << "- Scale Factor: " << scale_factor << std::endl;
			OPENSLAM_INFO << "- Initial Fast Threshold: " << default_fast_threshold << std::endl;
			OPENSLAM_INFO << "- Minimum Fast Threshold: " << min_fast_threshold << std::endl;

			stage_ = STAGE_FIRST_FRAME;//初始化的时候，设置处理帧为第一帧
			
		}

		TrackingMono::~TrackingMono()
		{
			delete cam_;
		}

		void TrackingMono::initialize()
		{

		}

		void TrackingMono::addImage(const cv::Mat& img, double timestamp)
		{
			// 初始化新帧
			cur_frame_ = std::make_shared<Frame>(cam_, img.clone(), timestamp, extractor_,orb_vocabulary_, is_rgb_order_);
			bool res = false;
			if (stage_ == STAGE_DEFAULT_FRAME)
				res = processFrame();
			else if (stage_ == STAGE_SECOND_FRAME)
				res = processSecondFrame();
			else if (stage_ == STAGE_FIRST_FRAME)
				res = processFirstFrame();

			//处理结束，将当前帧赋给上一帧
			last_frame_ = cur_frame_;

			//添加轨迹数据
			if (!cur_frame_->Tcw_.empty())
			{
				// 当前帧
				cv::Mat Tcr = cur_frame_->Tcw_*cur_frame_->ref_keyframe_->getPose();
				camera_trajectory_.list_frame_tcw.push_back(cur_frame_->Tcw_);
				camera_trajectory_.list_relative_frame_poses.push_back(Tcr);
				camera_trajectory_.list_ref_keyframes.push_back(cur_frame_->ref_keyframe_);
				camera_trajectory_.list_frame_times.push_back(cur_frame_->timestamp_);
				camera_trajectory_.list_is_lost.push_back(tracking_state_ == TRACKING_LOST);
			}
			else
			{
				// 如果帧数据丢失就添加上一帧的数据
				camera_trajectory_.list_frame_tcw.push_back(camera_trajectory_.list_frame_tcw.back());
				camera_trajectory_.list_relative_frame_poses.push_back(camera_trajectory_.list_relative_frame_poses.back());
				camera_trajectory_.list_ref_keyframes.push_back(camera_trajectory_.list_ref_keyframes.back());
				camera_trajectory_.list_frame_times.push_back(camera_trajectory_.list_frame_times.back());
				camera_trajectory_.list_is_lost.push_back(tracking_state_ == TRACKING_LOST);
			}
			//重置智能指针使用数
			cur_frame_.reset();
		}

		bool TrackingMono::processFirstFrame()
		{
			bool is_success = true;
			is_success = initializer_.addFirstFrame(cur_frame_);
			if (is_success)//如果初始化成功，则将当前帧转为关键帧，并计算BoW
			{
				KeyFrame *initkeyframe = new KeyFrame(*cur_frame_);
				init_keyframe_ = initkeyframe;
				init_keyframe_->computeBoW();
				//插入关键帧到地图中,释放放到地图中
				map_->addKeyframe(init_keyframe_);

				//给出当前帧的参考关键帧
				cur_frame_->ref_keyframe_ = initkeyframe;
				// 帧处理完，赋给上一帧
				last_keyframe_ = initkeyframe;
				// 将当前帧赋给参考帧，供局部优化使用
				ref_keyframe_ = initkeyframe;

				stage_ = STAGE_SECOND_FRAME;
			}
			return is_success;
		}

		bool TrackingMono::processSecondFrame()
		{
			bool is_success = true;
			is_success = initializer_.addSecondFrame(cur_frame_);
			if (is_success)//如果初始化成功，则将当前帧转为关键帧，并计算BoW
			{
				KeyFrame *cur_keyframe = new KeyFrame(*cur_frame_);
				cur_keyframe->computeBoW();
				//插入关键帧到地图中
				map_->addKeyframe(cur_keyframe);

				//将三维点分配到map point
				//创建map point分配到关键帧中
				std::vector<int> init_matchex = initializer_.init_matchex_;
				for (size_t i = 0; i < init_matchex.size(); i++)
				{
					if (init_matchex[i] < 0)
						continue;

					cv::Mat world_pos(initializer_.init_3d_points_[i]);//三角定位得到的世界坐标系中的点
					//将初始化得到的3d点创建 MapPoint.这边内存释放放到map中,将点与特征进行关联
					Feature *ref_feat = initializer_.ref_features_[i];
					Feature *cur_feat = initializer_.cur_features_[init_matchex[i]];
					MapPoint* map_point = new MapPoint(world_pos, cur_feat);
					ref_feat->addMapPointRef(map_point);
					cur_feat->addMapPointRef(map_point);
					map_point->addFeatureRef(ref_feat);
					map_point->addFeatureRef(cur_feat);
					map_point->computeDistinctiveDescriptors();
					map_point->updateNormalAndDepth();

					map_point->is_outlier_ = false;
					map_->addMapPoint(map_point);
					///这样就完成添加map point与特征的对应
				}

				OPENSLAM_INFO << "New Map created with " << map_->mapPointsInMap() << " points" << std::endl;

				// 计算中值深度,进行尺度处理
				float median_depth = init_keyframe_->computeSceneMedianDepth();
				float inv_median_depth = 1.0f / median_depth;

				if (median_depth < 0 || cur_keyframe->trackedMapPoints(1) < 100)
				{
					OPENSLAM_WARN << "Wrong initialization, reseting..." << std::endl;
					return false;
				}

				// 对外参的平移做尺度处理
				cv::Mat Tc2w = cur_keyframe->getCameraExternal();
				Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3)*inv_median_depth;
				cur_keyframe->setCameraExternal(Tc2w);

				// 对map point也进行尺度处理
				std::vector<MapPoint*> all_map_points = init_keyframe_->getMapPointMatches();
				for (size_t i = 0; i < all_map_points.size(); i++)
				{
					if (all_map_points[i])
					{
						MapPoint* map_point = all_map_points[i];
						map_point->setWorldPosition(map_point->getWorldPosition()*inv_median_depth);
					}
				}
				//给出当前帧的参考关键帧
				cur_frame_->ref_keyframe_ = cur_keyframe;
				// 帧处理完，赋给上一帧
				last_keyframe_ = cur_keyframe;
				// 将当前帧赋给参考帧，供局部优化使用
				ref_keyframe_ = cur_keyframe;
				
				vec_local_map_points_ = map_->getAllMapPoints();
				map_->setReferenceMapPoints(vec_local_map_points_);

				stage_ = STAGE_DEFAULT_FRAME;
				tracking_state_ == TRACKING_OK;
			}

			return is_success;
		}

		bool TrackingMono::processFrame()
		{
			bool is_ok = true;
			
			return is_ok;
		}
	}
}