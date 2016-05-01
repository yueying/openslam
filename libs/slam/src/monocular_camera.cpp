/*************************************************************************
 * 文件名： pinhole_camera
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/1
 *
 * 说明： 参考rpg_svo(https://github.com/uzh-rpg/rpg_svo)
 *************************************************************************/
#include <openslam/slam/monocular_camera.h>

namespace openslam
{
	namespace slam
	{
		// 考虑畸变参数k1,k2,p1,p2,k3
		MonocularCamera::MonocularCamera(double width, double height,
			double fx, double fy,
			double cx, double cy,
			double k1, double k2, double p1, double p2, double k3) :
			AbstractCamera(width, height, fx, fy, cx, cy, k1, k2, p1, p2, k3)
		{}

		MonocularCamera::~MonocularCamera(){}

	}
}
