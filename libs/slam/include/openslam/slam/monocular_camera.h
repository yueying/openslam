/*************************************************************************
 * 文件名： pinhole_camera
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/1
 *
 * 说明： 参考rpg_svo(https://github.com/uzh-rpg/rpg_svo)
 *************************************************************************/
#ifndef OPENSLAM_SLAM_PINHOLE_CAMERA_H_
#define OPENSLAM_SLAM_PINHOLE_CAMERA_H_

#include <openslam/slam/abstract_camera.h>

namespace openslam
{
	namespace slam
	{
		class SLAM_IMPEXP MonocularCamera:public AbstractCamera {

		public:
			// 考虑畸变参数k1,k2,p1,p2,k3
			MonocularCamera(double width, double height,
				double fx, double fy, double cx, double cy,
				double k1 = 0.0, double k2 = 0.0, double p1 = 0.0, double p2 = 0.0, double k3 = 0.0);

			~MonocularCamera();

			double getMapFactor() const
			{
				return -1.;
			}

			double getBaselineMultFx() const
			{
				return -1.;
			}
		};
	}
}

#endif // OPENSLAM_SLAM_PINHOLE_CAMERA_H_
