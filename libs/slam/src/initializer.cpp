#include <openslam/slam/initializer.h>
#include <thread>
#include <openslam/utils/notify.h>
#include <openslam/slam/orb_matcher.h>
#include <openslam/slam/feature.h>

namespace openslam
{
	namespace slam
	{
		Initializer::Initializer(float sigma, int iterations)
		{
			sigma_ = sigma;
			max_iter_num_ = iterations;
		}

		Initializer::~Initializer()
		{}

		void Initializer::reset()
		{
			ref_features_.clear();
			ref_frame_.reset();
		}

		bool Initializer::addFirstFrame(FramePtr ref_frame)
		{
			reset();
			ref_features_ = ref_frame->features_;
			if (ref_features_.size() < 100)
			{
				OPENSLAM_WARN << "First image has less than 100 features. Retry in more textured environment." << std::endl;
				return false;
			}
			int keypoints_num = ref_frame->getKeypointsNum();
			prev_matched_.resize(keypoints_num);
			for (size_t i = 0; i < keypoints_num; i++)
				prev_matched_[i] = ref_frame->features_[i]->undistored_keypoint_.pt;
			std::fill(init_matchex_.begin(), init_matchex_.end(), -1);
			// 赋值给属性并设置初始位姿
			ref_frame_ = ref_frame;
			ref_frame_->Tcw_ = cv::Mat::eye(4, 4, CV_32F);
			return true;
		}

		bool Initializer::addSecondFrame(FramePtr cur_frame)
		{
			cur_features_ = cur_frame->features_;
			if (cur_features_.size() < 100)
			{
				OPENSLAM_WARN << "Second image has less than 100 features. Retry in more textured environment." << std::endl;
				return false;
			}
			// 寻找对应匹配
			ORBmatcher matcher(0.9, true);
			int nmatches = matcher.searchForInitialization(ref_frame_, cur_frame, prev_matched_, init_matchex_, 100);
			// 检测是否有足够的匹配
			if (nmatches < 100)
			{
				OPENSLAM_WARN << "Matching number has less than 100. " << std::endl;
				return false;
			}
			cur_features_ = cur_frame->features_;
			// 给出当前帧与参考帧的匹配
			matches_ref_cur_.clear();
			matches_ref_cur_.reserve(cur_features_.size());
			is_matched_ref_.resize(ref_features_.size());
			for (size_t i = 0, iend = init_matchex_.size(); i < iend; i++)
			{
				if (init_matchex_[i] >= 0)
				{
					matches_ref_cur_.push_back(std::make_pair(i, init_matchex_[i]));
					is_matched_ref_[i] = true;
				}
				else
				{
					is_matched_ref_[i] = false;
				}
			}

			const int N = matches_ref_cur_.size();

			// 用于随机取8组数据的全部索引
			std::vector<size_t> all_indices;
			all_indices.reserve(N);
			std::vector<size_t> available_indices;

			for (int i = 0; i < N; i++)
			{
				all_indices.push_back(i);
			}

			// 生成一组8个点的数据集用于RANSAC的迭代
			ransac_sets_ = std::vector< std::vector<size_t> >(max_iter_num_, std::vector<size_t>(8, 0));
			// 这边随机改time(0)
			srand(time(0));

			for (int it = 0; it < max_iter_num_; it++)
			{
				available_indices = all_indices;
				// 在所有数据集中选择8组
				for (size_t j = 0; j < 8; j++)
				{
					int d = available_indices.size();
					int randi = int(((double)rand() / ((double)RAND_MAX + 1.0)) * d);

					int idx = available_indices[randi];

					ransac_sets_[it][j] = idx;

					available_indices[randi] = available_indices.back();
					available_indices.pop_back();
				}
			}

			// 启动线程用于并行计算基础矩阵和单应矩阵
			std::vector<bool> matches_inliers_H, matches_inliers_F;

			float SH, SF;
			cv::Mat H, F;

			std::thread threadH(&Initializer::findHomography, this, std::ref(matches_inliers_H), std::ref(SH), std::ref(H));
			std::thread threadF(&Initializer::findFundamental, this, std::ref(matches_inliers_F), std::ref(SF), std::ref(F));

			// 等待直到两个线程结束
			threadH.join();
			threadF.join();

			// 计算scores的比值
			float RH = SH / (SH + SF);

			cv::Mat R_cur_ref; //当前相机的旋转,相对于上一帧，上一帧为初始帧姿态为I单位阵
			cv::Mat t_cur_ref; // 当前相机的平移
			std::vector<bool> is_triangulated; // 初始化匹配当中，三角定位是否成功
			cv::Mat K = cur_frame->cam_->cvK();
			// 具体采用基础矩阵还是单应矩阵分解计算初始结构取决于ratio (0.40-0.45)
			if (RH > 0.40)
			{
				if (!reconstructH(matches_inliers_H, H, K, R_cur_ref, t_cur_ref, init_3d_points_, is_triangulated, 1.0, 50))
					return false;
			}
			else
			{
				if (!reconstructF(matches_inliers_F, F, K, R_cur_ref, t_cur_ref, init_3d_points_, is_triangulated, 1.0, 50))
					return false;
			}
			
			for (size_t i = 0, iend = init_matchex_.size(); i < iend; i++)
			{
				if (init_matchex_[i] >= 0 && !is_triangulated[i])
				{
					init_matchex_[i] = -1;
					nmatches--;
				}
			}
			OPENSLAM_INFO << "after triangulated the matches num :" << nmatches << std::endl;
			// 设置当前帧的位姿
			cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
			R_cur_ref.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
			t_cur_ref.copyTo(Tcw.rowRange(0, 3).col(3));
			cur_frame->Tcw_ = Tcw;
		
			return true;
		}

		void Initializer::findHomography(std::vector<bool> &matches_is_inliers, float &score, cv::Mat &H_cur_from_ref)
		{
			// 目前匹配的个数
			const int N = matches_ref_cur_.size();

			// 对点进行归一化变换
			std::vector<cv::Point2f> points_ref, points_cur;
			cv::Mat T_ref, T_cur;
			normalize(ref_features_, points_ref, T_ref);
			normalize(cur_features_, points_cur, T_cur);
			cv::Mat T_cur_inv = T_cur.inv();

			// 存储结果
			score = 0.0f;
			matches_is_inliers = std::vector<bool>(N, false);

			// 迭代变量
			std::vector<cv::Point2f> points_ref_cal(8);
			std::vector<cv::Point2f> points_cur_cal(8);
			cv::Mat H_cur_from_ref_tmp, H_ref_from_cur_tmp;// 临时计算出的单应矩阵
			std::vector<bool> current_is_inliers(N, false);

			float current_score;
			// 执行RANSAC的迭代
			for (int it = 0; it < max_iter_num_; it++)
			{
				// 对ransac的数据集进行处理
				for (size_t j = 0; j < 8; j++)
				{
					int idx = ransac_sets_[it][j];
					points_ref_cal[j] = points_ref[matches_ref_cur_[idx].first];
					points_cur_cal[j] = points_cur[matches_ref_cur_[idx].second];
				}
				// 通过归一化的点计算出的单应矩阵
				cv::Mat H = calcHFromMatches(points_ref_cal, points_cur_cal);
				// 解除归一化
				H_cur_from_ref_tmp = T_cur_inv*H*T_ref;
				H_ref_from_cur_tmp = H_cur_from_ref_tmp.inv();

				current_score = checkHomography(H_cur_from_ref_tmp, H_ref_from_cur_tmp, current_is_inliers, sigma_);

				if (current_score > score)
				{
					H_cur_from_ref = H_cur_from_ref_tmp.clone();
					matches_is_inliers = current_is_inliers;
					score = current_score;
				}
			}
		}

		void Initializer::findFundamental(std::vector<bool> &matches_is_inliers, float &score, cv::Mat &F21)
		{
			// 目前匹配的个数
			const int N = matches_is_inliers.size();

			// 对点进行归一化变换
			std::vector<cv::Point2f> points_ref, points_cur;
			cv::Mat T_ref, T_cur;
			normalize(ref_features_, points_ref, T_ref);
			normalize(cur_features_, points_cur, T_cur);
			cv::Mat T2t = T_cur.t();

			// 存储结果
			score = 0.0;
			matches_is_inliers = std::vector<bool>(N, false);

			// 迭代变量
			std::vector<cv::Point2f> points_ref_cal(8);
			std::vector<cv::Point2f> points_cur_cal(8);
			cv::Mat F_cur_from_ref_tmp;
			std::vector<bool> current_is_inliers(N, false);
			float current_score;

			// 执行RANSAC的迭代
			for (int it = 0; it < max_iter_num_; it++)
			{
				for (int j = 0; j < 8; j++)
				{
					int idx = ransac_sets_[it][j];

					points_ref_cal[j] = points_ref[matches_ref_cur_[idx].first];
					points_cur_cal[j] = points_cur[matches_ref_cur_[idx].second];
				}
				// 通过归一化的点计算出的基础矩阵
				cv::Mat F = calcFFromMatches(points_ref_cal, points_cur_cal);
				// 解除归一化
				F_cur_from_ref_tmp = T2t*F*T_ref;

				current_score = checkFundamental(F_cur_from_ref_tmp, current_is_inliers, sigma_);

				if (current_score > score)
				{
					F21 = F_cur_from_ref_tmp.clone();
					matches_is_inliers = current_is_inliers;
					score = current_score;
				}
			}
		}

		cv::Mat Initializer::calcHFromMatches(const std::vector<cv::Point2f> &points_ref, const std::vector<cv::Point2f> &points_cur)
		{
			/**
			* x = H y ，则对向量 x和Hy 进行叉乘为0，即：
			*
			* | 0 -1  v2|   |a b c|   |u1|    |0|
			* | 1  0 -u2| * |d e f| * |v1| =  |0|
			* |-v2  u2 0|   |g h 1|   |1 |    |0|
			*
			* 矩阵化简得：
			*
			* (-d+v2*g)*u1    + (-e+v2*h)*v1 + -f+v2          |0|
			* (a-u2*g)*u1     + (b-u2*h)*v1  + c-u2         = |0|
			* (-v2*a+u2*d)*u1 + (-v2*b+u2*e)*v1 + -v2*c+u2*f  |0|
			*
			* 0*a + 0*b + 0*c - u1*d - v1*e - 1*f + v2*u1*g + v2*v1*h + v2 = 0
			* u1*a+v1*b + 1*c + 0*d  + 0*e   +0*f - u2*u1*g - u2*v1*h - u2 = 0
			* -v2*u1*a -v2*v1*b -v2*c +u2*u1*d +u2*v1*e +u2*f +0*g +0*h + 0 = 0
			*/
			const int N = points_ref.size();

			cv::Mat A(2 * N, 9, CV_32F);

			for (int i = 0; i < N; i++)
			{
				const float u1 = points_ref[i].x;
				const float v1 = points_ref[i].y;
				const float u2 = points_cur[i].x;
				const float v2 = points_cur[i].y;

				A.at<float>(2 * i, 0) = 0.0;
				A.at<float>(2 * i, 1) = 0.0;
				A.at<float>(2 * i, 2) = 0.0;
				A.at<float>(2 * i, 3) = -u1;
				A.at<float>(2 * i, 4) = -v1;
				A.at<float>(2 * i, 5) = -1;
				A.at<float>(2 * i, 6) = v2*u1;
				A.at<float>(2 * i, 7) = v2*v1;
				A.at<float>(2 * i, 8) = v2;

				A.at<float>(2 * i + 1, 0) = u1;
				A.at<float>(2 * i + 1, 1) = v1;
				A.at<float>(2 * i + 1, 2) = 1;
				A.at<float>(2 * i + 1, 3) = 0.0;
				A.at<float>(2 * i + 1, 4) = 0.0;
				A.at<float>(2 * i + 1, 5) = 0.0;
				A.at<float>(2 * i + 1, 6) = -u2*u1;
				A.at<float>(2 * i + 1, 7) = -u2*v1;
				A.at<float>(2 * i + 1, 8) = -u2;

			}

			cv::Mat u, w, vt;
			// 通过svd进行最小二乘求解
			cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

			return vt.row(8).reshape(0, 3);
		}

		cv::Mat Initializer::calcFFromMatches(const std::vector<cv::Point2f> &points_ref, const std::vector<cv::Point2f> &points_cur)
		{
			/**
			*      构建基础矩阵的约束方程，给定一对点对应m=(u1,v1,1)T,m'=(u2,v2,1)T
			*  	   满足基础矩阵F m'TFm=0,令F=(f_ij),则约束方程可以化简为：
			*  	    u2u1f_11+u2v1f_12+u2f_13+v2u1f_21+v2v1f_22+v2f_23+u1f_31+v1f_32+f_33=0
			*  	    令f = (f_11,f_12,f_13,f_21,f_22,f_23,f_31,f_32,f_33)
			*  	    则(u2u1,u2v1,u2,v2u1,v2v1,v2,u1,v1,1)f=0;
			*  	    这样，给定N个对应点就可以得到线性方程组Af=0
			*  	    A就是一个N*9的矩阵，由于基础矩阵是非零的，所以f是一个非零向量，即
			*  	    线性方程组有非零解，另外基础矩阵的秩为2，重要的约束条件
			*/
			const int N = points_ref.size();

			cv::Mat A(N, 9, CV_32F);

			for (int i = 0; i < N; i++)
			{
				const float u1 = points_ref[i].x;
				const float v1 = points_ref[i].y;
				const float u2 = points_cur[i].x;
				const float v2 = points_cur[i].y;

				A.at<float>(i, 0) = u2*u1;
				A.at<float>(i, 1) = u2*v1;
				A.at<float>(i, 2) = u2;
				A.at<float>(i, 3) = v2*u1;
				A.at<float>(i, 4) = v2*v1;
				A.at<float>(i, 5) = v2;
				A.at<float>(i, 6) = u1;
				A.at<float>(i, 7) = v1;
				A.at<float>(i, 8) = 1;
			}

			cv::Mat u, w, vt;

			cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

			cv::Mat Fpre = vt.row(8).reshape(0, 3);

			cv::SVDecomp(Fpre, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

			w.at<float>(2) = 0;

			return  u*cv::Mat::diag(w)*vt;
		}

		float Initializer::checkHomography(const cv::Mat &H_cur_from_ref, const cv::Mat &H_ref_from_cur,
			std::vector<bool> &matches_is_inliers, float sigma)
		{
			const int N = matches_ref_cur_.size();

			const float h11 = H_cur_from_ref.at<float>(0, 0);
			const float h12 = H_cur_from_ref.at<float>(0, 1);
			const float h13 = H_cur_from_ref.at<float>(0, 2);
			const float h21 = H_cur_from_ref.at<float>(1, 0);
			const float h22 = H_cur_from_ref.at<float>(1, 1);
			const float h23 = H_cur_from_ref.at<float>(1, 2);
			const float h31 = H_cur_from_ref.at<float>(2, 0);
			const float h32 = H_cur_from_ref.at<float>(2, 1);
			const float h33 = H_cur_from_ref.at<float>(2, 2);

			const float h11inv = H_ref_from_cur.at<float>(0, 0);
			const float h12inv = H_ref_from_cur.at<float>(0, 1);
			const float h13inv = H_ref_from_cur.at<float>(0, 2);
			const float h21inv = H_ref_from_cur.at<float>(1, 0);
			const float h22inv = H_ref_from_cur.at<float>(1, 1);
			const float h23inv = H_ref_from_cur.at<float>(1, 2);
			const float h31inv = H_ref_from_cur.at<float>(2, 0);
			const float h32inv = H_ref_from_cur.at<float>(2, 1);
			const float h33inv = H_ref_from_cur.at<float>(2, 2);

			matches_is_inliers.resize(N);

			float score = 0;

			const float th = 5.991;

			const float inv_sigma_square = 1.0 / (sigma*sigma);

			for (int i = 0; i < N; i++)
			{
				bool is_in = true;

				const cv::KeyPoint &kp1 = ref_features_[matches_ref_cur_[i].first]->undistored_keypoint_;
				const cv::KeyPoint &kp2 = cur_features_[matches_ref_cur_[i].second]->undistored_keypoint_;

				const float u1 = kp1.pt.x;
				const float v1 = kp1.pt.y;
				const float u2 = kp2.pt.x;
				const float v2 = kp2.pt.y;

				// 在第一幅图像上进行重投影计算误差
				// x2in1 = H_ref_from_cur*x2

				const float w2in1inv = 1.0 / (h31inv*u2 + h32inv*v2 + h33inv);
				const float u2in1 = (h11inv*u2 + h12inv*v2 + h13inv)*w2in1inv;
				const float v2in1 = (h21inv*u2 + h22inv*v2 + h23inv)*w2in1inv;

				const float square_dist1 = (u1 - u2in1)*(u1 - u2in1) + (v1 - v2in1)*(v1 - v2in1);

				const float chi_square1 = square_dist1*inv_sigma_square;

				if (chi_square1 > th)
					is_in = false;
				else
					score += th - chi_square1;

				// 在第二幅图像上进行重投影计算误差
				// x1in2 = H_cur_from_ref*x1

				const float w1in2inv = 1.0 / (h31*u1 + h32*v1 + h33);
				const float u1in2 = (h11*u1 + h12*v1 + h13)*w1in2inv;
				const float v1in2 = (h21*u1 + h22*v1 + h23)*w1in2inv;

				const float square_dist2 = (u2 - u1in2)*(u2 - u1in2) + (v2 - v1in2)*(v2 - v1in2);

				const float chi_square2 = square_dist2*inv_sigma_square;

				if (chi_square2 > th)
					is_in = false;
				else
					score += th - chi_square2;

				if (is_in)
					matches_is_inliers[i] = true;
				else
					matches_is_inliers[i] = false;
			}

			return score;
		}

		float Initializer::checkFundamental(const cv::Mat &F21, std::vector<bool> &matches_is_inliers, float sigma)
		{
			const int N = matches_ref_cur_.size();

			const float f11 = F21.at<float>(0, 0);
			const float f12 = F21.at<float>(0, 1);
			const float f13 = F21.at<float>(0, 2);
			const float f21 = F21.at<float>(1, 0);
			const float f22 = F21.at<float>(1, 1);
			const float f23 = F21.at<float>(1, 2);
			const float f31 = F21.at<float>(2, 0);
			const float f32 = F21.at<float>(2, 1);
			const float f33 = F21.at<float>(2, 2);

			matches_is_inliers.resize(N);

			float score = 0;

			const float th = 3.841f;
			const float th_score = 5.991f;

			const float inv_sigma_square = 1.0f / (sigma*sigma);

			for (int i = 0; i < N; i++)
			{
				bool is_in = true;

				const cv::KeyPoint &kp1 = ref_features_[matches_ref_cur_[i].first]->undistored_keypoint_;
				const cv::KeyPoint &kp2 = cur_features_[matches_ref_cur_[i].second]->undistored_keypoint_;

				const float u1 = kp1.pt.x;
				const float v1 = kp1.pt.y;
				const float u2 = kp2.pt.x;
				const float v2 = kp2.pt.y;

				// 在第一幅图像进行重投影计算
				// l2=F21x1=(a2,b2,c2)
				const float a2 = f11*u1 + f12*v1 + f13;
				const float b2 = f21*u1 + f22*v1 + f23;
				const float c2 = f31*u1 + f32*v1 + f33;

				const float num2 = a2*u2 + b2*v2 + c2;

				const float square_dist1 = num2*num2 / (a2*a2 + b2*b2);

				const float chi_square1 = square_dist1*inv_sigma_square;

				if (chi_square1 > th)
					is_in = false;
				else
					score += th_score - chi_square1;

				// 在第二幅图像进行重投影计算
				// l1 =x2tF21=(a1,b1,c1)

				const float a1 = f11*u2 + f21*v2 + f31;
				const float b1 = f12*u2 + f22*v2 + f32;
				const float c1 = f13*u2 + f23*v2 + f33;

				const float num1 = a1*u1 + b1*v1 + c1;

				const float square_dist2 = num1*num1 / (a1*a1 + b1*b1);

				const float chi_square2 = square_dist2*inv_sigma_square;

				if (chi_square2 > th)
					is_in = false;
				else
					score += th_score - chi_square2;

				if (is_in)
					matches_is_inliers[i] = true;
				else
					matches_is_inliers[i] = false;
			}

			return score;
		}

		bool Initializer::reconstructF(std::vector<bool> &matches_is_inliers, cv::Mat &F21, cv::Mat &K,
			cv::Mat &R_cur_ref, cv::Mat &t_cur_ref, std::vector<cv::Point3f> &vec_point3d,
			std::vector<bool> &is_triangulated, float min_parallax, int min_triangulated)
		{
			int N = 0;
			for (size_t i = 0, iend = matches_is_inliers.size(); i < iend; i++)
				if (matches_is_inliers[i])
					N++;

			// 根据基础矩阵计算本质矩阵
			cv::Mat E21 = K.t()*F21*K;

			cv::Mat R1, R2, t;

			// 对本质矩阵进行分解,有四组可能解
			decomposeE(E21, R1, R2, t);

			cv::Mat t1 = t;
			cv::Mat t2 = -t;

			// 对4组可能的解进行检测，判断计算出的三维点在两个相机前面
			std::vector<cv::Point3f> vec_point3d1, vec_point3d2, vec_point3d3, vec_point3d4;
			std::vector<bool> is_triangulated1, is_triangulated2, is_triangulated3, is_triangulated4;
			float parallax1, parallax2, parallax3, parallax4;
			float sigma2 = sigma_*sigma_;
			int good1_num = checkRt(R1, t1, ref_features_, cur_features_, matches_ref_cur_, matches_is_inliers, K, vec_point3d1, 4.0*sigma2, is_triangulated1, parallax1);
			int good2_num = checkRt(R2, t1, ref_features_, cur_features_, matches_ref_cur_, matches_is_inliers, K, vec_point3d2, 4.0*sigma2, is_triangulated2, parallax2);
			int good3_num = checkRt(R1, t2, ref_features_, cur_features_, matches_ref_cur_, matches_is_inliers, K, vec_point3d3, 4.0*sigma2, is_triangulated3, parallax3);
			int good4_num = checkRt(R2, t2, ref_features_, cur_features_, matches_ref_cur_, matches_is_inliers, K, vec_point3d4, 4.0*sigma2, is_triangulated4, parallax4);

			int max_good = std::max(good1_num, std::max(good2_num, std::max(good3_num, good4_num)));

			R_cur_ref = cv::Mat();
			t_cur_ref = cv::Mat();

			int min_good_num = std::max(static_cast<int>(0.9*N), min_triangulated);

			int nsimilar = 0;
			if (good1_num > 0.7*max_good)
				nsimilar++;
			if (good2_num > 0.7*max_good)
				nsimilar++;
			if (good3_num > 0.7*max_good)
				nsimilar++;
			if (good4_num > 0.7*max_good)
				nsimilar++;

			// 如果还不能区分哪组结果比较好，或者没有足够的三角定位的点，则拒绝初始化
			if (max_good < min_good_num || nsimilar>1)
			{
				return false;
			}

			// 有足够的视差进行初始化
			if (max_good == good1_num)
			{
				if (parallax1 > min_parallax)
				{
					vec_point3d = vec_point3d1;
					is_triangulated = is_triangulated1;

					R1.copyTo(R_cur_ref);
					t1.copyTo(t_cur_ref);
					return true;
				}
			}
			else if (max_good == good2_num)
			{
				if (parallax2 > min_parallax)
				{
					vec_point3d = vec_point3d2;
					is_triangulated = is_triangulated2;

					R2.copyTo(R_cur_ref);
					t1.copyTo(t_cur_ref);
					return true;
				}
			}
			else if (max_good == good3_num)
			{
				if (parallax3 > min_parallax)
				{
					vec_point3d = vec_point3d3;
					is_triangulated = is_triangulated3;

					R1.copyTo(R_cur_ref);
					t2.copyTo(t_cur_ref);
					return true;
				}
			}
			else if (max_good == good4_num)
			{
				if (parallax4 > min_parallax)
				{
					vec_point3d = vec_point3d4;
					is_triangulated = is_triangulated4;

					R2.copyTo(R_cur_ref);
					t2.copyTo(t_cur_ref);
					return true;
				}
			}

			return false;
		}

		bool Initializer::reconstructH(std::vector<bool> &matches_is_inliers, cv::Mat &H_cur_from_ref, cv::Mat &K,
			cv::Mat &R_cur_ref, cv::Mat &t_cur_ref, std::vector<cv::Point3f> &vec_point3d, std::vector<bool> &is_triangulated,
			float min_parallax, int min_triangulated)
		{
			int N = 0;
			for (size_t i = 0, iend = matches_is_inliers.size(); i < iend; i++)
			{
				if (matches_is_inliers[i])
					N++;
			}

			// 单应矩阵分解，通过svd分解，根据特征值条件，构建了8组理论解。具体参考论文 Faugeras et al.
			// Motion and structure from motion in a piecewise planar environment.
			// International Journal of Pattern Recognition and Artificial Intelligence, 1988
			cv::Mat invK = K.inv();
			cv::Mat A = invK*H_cur_from_ref*K;

			cv::Mat U, w, Vt, V;
			cv::SVD::compute(A, w, U, Vt, cv::SVD::FULL_UV);
			V = Vt.t();

			float s = cv::determinant(U)*cv::determinant(Vt);

			float d1 = w.at<float>(0);
			float d2 = w.at<float>(1);
			float d3 = w.at<float>(2);

			if (d1 / d2 < 1.00001 || d2 / d3 < 1.00001)
			{
				return false;
			}

			std::vector<cv::Mat> vR, vt, vn;
			vR.reserve(8);
			vt.reserve(8);
			vn.reserve(8);

			//n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
			float aux1 = sqrt((d1*d1 - d2*d2) / (d1*d1 - d3*d3));
			float aux3 = sqrt((d2*d2 - d3*d3) / (d1*d1 - d3*d3));
			float x1[] = { aux1, aux1, -aux1, -aux1 };
			float x3[] = { aux3, -aux3, aux3, -aux3 };

			//case d'=d2
			float aux_stheta = sqrt((d1*d1 - d2*d2)*(d2*d2 - d3*d3)) / ((d1 + d3)*d2);

			float ctheta = (d2*d2 + d1*d3) / ((d1 + d3)*d2);
			float stheta[] = { aux_stheta, -aux_stheta, -aux_stheta, aux_stheta };

			for (int i = 0; i < 4; i++)
			{
				cv::Mat Rp = cv::Mat::eye(3, 3, CV_32F);
				Rp.at<float>(0, 0) = ctheta;
				Rp.at<float>(0, 2) = -stheta[i];
				Rp.at<float>(2, 0) = stheta[i];
				Rp.at<float>(2, 2) = ctheta;

				cv::Mat R = s*U*Rp*Vt;
				vR.push_back(R);

				cv::Mat tp(3, 1, CV_32F);
				tp.at<float>(0) = x1[i];
				tp.at<float>(1) = 0;
				tp.at<float>(2) = -x3[i];
				tp *= d1 - d3;

				cv::Mat t = U*tp;
				vt.push_back(t / cv::norm(t));

				cv::Mat np(3, 1, CV_32F);
				np.at<float>(0) = x1[i];
				np.at<float>(1) = 0;
				np.at<float>(2) = x3[i];

				cv::Mat n = V*np;
				if (n.at<float>(2) < 0)
					n = -n;
				vn.push_back(n);
			}

			//case d'=-d2
			float aux_sphi = sqrt((d1*d1 - d2*d2)*(d2*d2 - d3*d3)) / ((d1 - d3)*d2);

			float cphi = (d1*d3 - d2*d2) / ((d1 - d3)*d2);
			float sphi[] = { aux_sphi, -aux_sphi, -aux_sphi, aux_sphi };

			for (int i = 0; i < 4; i++)
			{
				cv::Mat Rp = cv::Mat::eye(3, 3, CV_32F);
				Rp.at<float>(0, 0) = cphi;
				Rp.at<float>(0, 2) = sphi[i];
				Rp.at<float>(1, 1) = -1;
				Rp.at<float>(2, 0) = sphi[i];
				Rp.at<float>(2, 2) = -cphi;

				cv::Mat R = s*U*Rp*Vt;
				vR.push_back(R);

				cv::Mat tp(3, 1, CV_32F);
				tp.at<float>(0) = x1[i];
				tp.at<float>(1) = 0;
				tp.at<float>(2) = x3[i];
				tp *= d1 + d3;

				cv::Mat t = U*tp;
				vt.push_back(t / cv::norm(t));

				cv::Mat np(3, 1, CV_32F);
				np.at<float>(0) = x1[i];
				np.at<float>(1) = 0;
				np.at<float>(2) = x3[i];

				cv::Mat n = V*np;
				if (n.at<float>(2) < 0)
					n = -n;
				vn.push_back(n);
			}


			int best_good = 0;
			int second_best_good = 0;
			int best_solution_idx = -1;
			float best_parallax = -1;
			std::vector<cv::Point3f> best_point3d;
			std::vector<bool> best_triangulated;

			// Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
			// We reconstruct all hypotheses and check in terms of triangulated points and parallax
			for (size_t i = 0; i < 8; i++)
			{
				float parallaxi;
				std::vector<cv::Point3f> vec_point3d_tmp;
				std::vector<bool> vec_is_triangulated_tmp;
				int good_num = checkRt(vR[i], vt[i], ref_features_, cur_features_, matches_ref_cur_, matches_is_inliers, K, vec_point3d_tmp, 4.0*sigma_*sigma_, vec_is_triangulated_tmp, parallaxi);

				if (good_num > best_good)
				{
					second_best_good = best_good;
					best_good = good_num;
					best_solution_idx = i;
					best_parallax = parallaxi;
					best_point3d = vec_point3d_tmp;
					best_triangulated = vec_is_triangulated_tmp;
				}
				else if (good_num > second_best_good)
				{
					second_best_good = good_num;
				}
			}


			if (second_best_good<0.75*best_good && best_parallax >= min_parallax && best_good>min_triangulated && best_good > 0.9*N)
			{
				vR[best_solution_idx].copyTo(R_cur_ref);
				vt[best_solution_idx].copyTo(t_cur_ref);
				vec_point3d = best_point3d;
				is_triangulated = best_triangulated;

				return true;
			}

			return false;
		}

		void Initializer::triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
		{
			cv::Mat A(4, 4, CV_32F);
			/** P=(p1,p2,p3)T*X = s(u,v)T ，T是转置
			*/
			A.row(0) = kp1.pt.x*P1.row(2) - P1.row(0);
			A.row(1) = kp1.pt.y*P1.row(2) - P1.row(1);
			A.row(2) = kp2.pt.x*P2.row(2) - P2.row(0);
			A.row(3) = kp2.pt.y*P2.row(2) - P2.row(1);

			cv::Mat u, w, vt;
			cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
			x3D = vt.row(3).t();
			x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
		}

		void Initializer::normalize(const std::vector<Feature *> &features, std::vector<cv::Point2f> &normalized_points, cv::Mat &T)
		{
			// 特征点坐标均值
			float mean_x = 0;
			float mean_y = 0;
			const int N = features.size();

			normalized_points.resize(N);

			for (int i = 0; i < N; i++)
			{
				mean_x += features[i]->undistored_keypoint_.pt.x;
				mean_y += features[i]->undistored_keypoint_.pt.y;
			}

			mean_x = mean_x / N;
			mean_y = mean_y / N;

			float mean_dev_x = 0;
			float mean_dev_y = 0;

			for (int i = 0; i < N; i++)
			{
				normalized_points[i].x = features[i]->undistored_keypoint_.pt.x - mean_x;
				normalized_points[i].y = features[i]->undistored_keypoint_.pt.y - mean_y;
				// TODO:这边归一化再考虑一下，是否计算方差
				mean_dev_x += fabs(normalized_points[i].x);
				mean_dev_y += fabs(normalized_points[i].y);
			}

			mean_dev_x = mean_dev_x / N;
			mean_dev_y = mean_dev_y / N;

			float scale_x = 1.0 / mean_dev_x;
			float scale_y = 1.0 / mean_dev_y;

			for (int i = 0; i < N; i++)
			{
				normalized_points[i].x = normalized_points[i].x * scale_x;
				normalized_points[i].y = normalized_points[i].y * scale_y;
			}

			T = cv::Mat::eye(3, 3, CV_32F);
			T.at<float>(0, 0) = scale_x;
			T.at<float>(1, 1) = scale_y;
			T.at<float>(0, 2) = -mean_x*scale_x;
			T.at<float>(1, 2) = -mean_y*scale_y;
		}

		int Initializer::checkRt(const cv::Mat &R, const cv::Mat &t,
			const std::vector<Feature *> &vec_features1, const std::vector<Feature *> &vec_features2,
			const std::vector<Match> &matches_ref_cur, std::vector<bool> &matches_is_inliers,
			const cv::Mat &K, std::vector<cv::Point3f> &vec_point3d, float th2, std::vector<bool> &vec_is_good, float &parallax)
		{
			// 相机参数
			const float fx = K.at<float>(0, 0);
			const float fy = K.at<float>(1, 1);
			const float cx = K.at<float>(0, 2);
			const float cy = K.at<float>(1, 2);

			vec_is_good = std::vector<bool>(vec_features1.size(), false);
			vec_point3d.resize(vec_features1.size());

			std::vector<float> vec_cos_parallax;
			vec_cos_parallax.reserve(vec_features1.size());

			// 相机1的投影矩阵 K[I|0]
			cv::Mat P1(3, 4, CV_32F, cv::Scalar(0));
			K.copyTo(P1.rowRange(0, 3).colRange(0, 3));

			cv::Mat O1 = cv::Mat::zeros(3, 1, CV_32F);//相机1位置

			// 相机2的投影矩阵 K[R|t]
			cv::Mat P2(3, 4, CV_32F);
			R.copyTo(P2.rowRange(0, 3).colRange(0, 3));
			t.copyTo(P2.rowRange(0, 3).col(3));
			P2 = K*P2;

			cv::Mat O2 = -R.t()*t;//相机2的位置

			int good_num = 0;

			for (size_t i = 0, iend = matches_ref_cur.size(); i < iend; i++)
			{
				if (!matches_is_inliers[i])
					continue;

				const cv::KeyPoint &kp1 = vec_features1[matches_ref_cur[i].first]->undistored_keypoint_;
				const cv::KeyPoint &kp2 = vec_features2[matches_ref_cur[i].second]->undistored_keypoint_;
				cv::Mat p3dC1;

				triangulate(kp1, kp2, P1, P2, p3dC1);

				if (!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
				{
					vec_is_good[matches_ref_cur[i].first] = false;
					continue;
				}

				// 计算视差
				cv::Mat normal1 = p3dC1 - O1;
				float dist1 = cv::norm(normal1);

				cv::Mat normal2 = p3dC1 - O2;
				float dist2 = cv::norm(normal2);

				float cos_parallax = normal1.dot(normal2) / (dist1*dist2);

				// 检测z必须大于0，即在相机的前面，另外保证视差不能太小
				if (p3dC1.at<float>(2) <= 0 && cos_parallax < 0.99998)
					continue;

				// 检测点是否在另外一个相机的前面，以及视差不能太小
				cv::Mat p3dC2 = R*p3dC1 + t;

				if (p3dC2.at<float>(2) <= 0 && cos_parallax < 0.99998)
					continue;

				// 检测第一幅图像上计算重投影误差
				float im1x, im1y;
				float invZ1 = 1.0 / p3dC1.at<float>(2);
				im1x = fx*p3dC1.at<float>(0)*invZ1 + cx;
				im1y = fy*p3dC1.at<float>(1)*invZ1 + cy;

				float square_error1 = (im1x - kp1.pt.x)*(im1x - kp1.pt.x) + (im1y - kp1.pt.y)*(im1y - kp1.pt.y);

				if (square_error1 > th2)
					continue;

				// 检测第二幅图像的重投影误差
				float im2x, im2y;
				float invZ2 = 1.0 / p3dC2.at<float>(2);
				im2x = fx*p3dC2.at<float>(0)*invZ2 + cx;
				im2y = fy*p3dC2.at<float>(1)*invZ2 + cy;

				float square_error2 = (im2x - kp2.pt.x)*(im2x - kp2.pt.x) + (im2y - kp2.pt.y)*(im2y - kp2.pt.y);

				if (square_error2 > th2)
					continue;

				vec_cos_parallax.push_back(cos_parallax);
				vec_point3d[matches_ref_cur[i].first] = cv::Point3f(p3dC1.at<float>(0), p3dC1.at<float>(1), p3dC1.at<float>(2));
				good_num++;

				if (cos_parallax < 0.99998)
					vec_is_good[matches_ref_cur[i].first] = true;
			}

			if (good_num > 0)
			{
				std::sort(vec_cos_parallax.begin(), vec_cos_parallax.end());

				size_t idx = std::min(50, int(vec_cos_parallax.size() - 1));
				parallax = acos(vec_cos_parallax[idx]) * 180 / CV_PI;
			}
			else
				parallax = 0;

			return good_num;
		}

		void Initializer::decomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
		{
			cv::Mat u, w, vt;
			cv::SVD::compute(E, w, u, vt);

			u.col(2).copyTo(t);
			t = t / cv::norm(t);

			cv::Mat W(3, 3, CV_32F, cv::Scalar(0));
			W.at<float>(0, 1) = -1;
			W.at<float>(1, 0) = 1;
			W.at<float>(2, 2) = 1;

			R1 = u*W*vt;
			if (cv::determinant(R1) < 0)
				R1 = -R1;

			R2 = u*W.t()*vt;
			if (cv::determinant(R2) < 0)
				R2 = -R2;
		}
	}
}