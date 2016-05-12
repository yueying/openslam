#ifndef OPENSLAM_SLAM_OPTIMIZER_H_
#define OPENSLAM_SLAM_OPTIMIZER_H_

#include <openslam/slam/link_pragmas.h>
#include <g2o/types/types_seven_dof_expmap.h>

#include <openslam/slam/map.h>
#include <openslam/slam/map_point.h>
#include <openslam/slam/keyframe.h>
#include <openslam/slam/frame.h>

namespace openslam
{
	namespace slam
	{

		class SLAM_IMPEXP Optimizer
		{
		public:
			void static bundleAdjustment(const std::vector<KeyFrame *> &vec_key_frame, const std::vector<MapPoint*> &vec_map_points,
				int iter_num = 5, bool *stop_flag = NULL, const unsigned long loop_keyframe_num = 0,
				const bool is_robust = true);

			void static globalBundleAdjustemnt(Map* map, int iter_num = 5, bool *stop_flag = NULL,
			const unsigned long loop_keyframe_num = 0, const bool is_robust = true);

			void static localBundleAdjustment(KeyFrame * keyframe, bool *stop_flag, Map *map);

			int static poseOptimization(FramePtr frame);

		};
	}
}


#endif // OPENSLAM_SLAM_OPTIMIZER_H_
