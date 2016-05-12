/*************************************************************************
* 文件名： tracking
*
* 作者： 冯兵
* 邮件： fengbing123@gmail.com
* 时间： 2016/4/9
*
* 说明： 参考rpg_svo,orb_slam2
*************************************************************************/
#ifndef OPENSLAM_SLAM_TRACKING_H_
#define OPENSLAM_SLAM_TRACKING_H_
#include <openslam/slam/frame.h>
#include <openslam/slam/map.h>

namespace openslam
{
	namespace slam
	{
		/** 跟踪类，主要用于接收帧并计算与之对应的相机姿态，另外决定何时插入一个关键帧，创建一个新
		*   的map point，还有如果跟踪失败执行重定位。
		*/
		class SLAM_IMPEXP TrackingBase
		{
		public:
			/** 跟踪状态
			*/
			enum TrackingState
			{
				TRACKING_SYSTEM_NOT_READY = -1,//状态主要用于导入词汇库的过程
				TRACKING_NO_IMAGES_YET = 0,
				TRACKING_NOT_INITIALIZED = 1,
				TRACKING_OK = 2,
				TRACKING_LOST = 3
			};

			/**给出跟踪的配置参数，不同的传感器配置值不一致*/
			struct Options
			{
				float track_motion_model_matcher_nnratio;//!<在根据运动模型跟踪的时候，匹配最小距离与次最小距离直接的比值阈值
				float track_ref_keyframe_matcher_nnratio;//!<对参考关键帧跟踪的时候，匹配最小距离与次最小距离直接的比值阈值
				float projection_threshold;//!<投影匹配搜索的窗体半价阈值，单目与双目不一致

				Options() :projection_threshold(15.f),
					track_motion_model_matcher_nnratio(0.9),
					track_ref_keyframe_matcher_nnratio(0.7)
				{}
			}options_;

			struct CameraTrajectory
			{
				std::list<cv::Mat> list_frame_tcw;
				std::list<cv::Mat> list_relative_frame_poses;
				std::list<KeyFrame *> list_ref_keyframes;
				std::list<double> list_frame_times;
				std::list<bool> list_is_lost;
			}camera_trajectory_;

			TrackingBase(ORBVocabulary *voc);
			virtual ~TrackingBase();
			/**不同的传感器初始化不一致*/
			virtual void initialize() = 0;

			/**输入一幅图像，给单目的接口*/
			virtual void addImage(const cv::Mat& img, double timestamp) = 0;

			/**跟踪线程执行的主函数*/
			void track();

			/**根据计算的运动模型进行跟踪*/
			bool trackWithMotionModel();

			/**根据参考关键帧进行跟踪*/
			bool trackReferenceKeyFrame();

			bool trackLocalMap();

			bool relocalization();
		protected:
			/**设置上一帧的位姿，防止丢失*/
			void updateLastFrame();

			void updateLocalMap();
			void updateLocalPoints();
			void updateLocalKeyFrames();
		protected:
			TrackingState tracking_state_;//!<表示设置的跟踪状态
			Map  *map_;
			ORBVocabulary *orb_vocabulary_;
			bool is_only_tracking_;//!< 如果是true,则表明只执行定位
			bool is_vo_;//!< 表明是否是视觉里程计		
			KeyFrame* init_keyframe_;//!< 初始关键帧
			FramePtr  cur_frame_;      //!< 当前帧
			bool is_rgb_order_;//!<图像顺序，true图像是RGB的顺序，false是BGR的顺序
			cv::Mat motion_model_;//!< 运动模型

			KeyFrame* ref_keyframe_;//!<局部地图中使用参考帧
			KeyFrame* last_keyframe_;//!< 上一个关键帧
			FramePtr last_frame_;//!<上一帧

			std::vector<MapPoint*> vec_local_map_points_;//!<跟踪对应的mappoint，主要用于可视化显示
		};
	}
}

#endif // OPENSLAM_SLAM_TRACKING_H_