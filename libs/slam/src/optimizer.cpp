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

namespace openslam
{
	namespace slam
	{

	}
}
