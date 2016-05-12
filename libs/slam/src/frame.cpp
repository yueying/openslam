#include <openslam/slam/frame.h>
#include <openslam/slam/feature.h>

namespace openslam
{
	namespace slam
	{
		long unsigned int Frame::frame_counter_ = 0;
		bool Frame::is_initial_computations_ = true;
		float Frame::min_bound_x_, Frame::min_bound_y_, Frame::max_bound_x_, Frame::max_bound_y_;
		float Frame::grid_element_height_, Frame::grid_element_width_;

		Frame::Frame(AbstractCamera* cam, const cv::Mat& img, double timestamp,
			ORBextractor* extractor, ORBVocabulary* voc, bool is_rgb_order) :
			id_(frame_counter_++),
			cam_(cam),
			img_(img),
			timestamp_(timestamp),
			extractor_(extractor),
			orb_vocabulary_(voc),
			is_rgb_order_(is_rgb_order)
		{
			// 获取尺度相关参数
			levels_num_ = extractor_->getLevels();
			scale_factor_ = extractor_->getScaleFactor();

			prepareImage(img, gray_img_);
			// 帧初始化的进行特征检测，初始化特征及特征数
			extractORB(gray_img_);

			// 主要用于第一次计算
			if (is_initial_computations_)
			{
				computeImageBounds(gray_img_);

				grid_element_height_ = (max_bound_x_ - min_bound_x_) / static_cast<float>(FRAME_GRID_COLS);
				grid_element_width_ = (max_bound_y_ - min_bound_y_) / static_cast<float>(FRAME_GRID_ROWS);

				is_initial_computations_ = false;
			}
			assignFeaturesToGrid();
		}

		Frame::Frame(const Frame &frame) :
			id_(frame.id_),
			timestamp_(frame.timestamp_),
			cam_(frame.cam_),
			img_(frame.img_.clone()),
			gray_img_(frame.gray_img_.clone()),
			is_rgb_order_(frame.is_rgb_order_),
			scale_factor_(frame.scale_factor_),
			Tcw_(frame.Tcw_.clone()),
			extractor_(frame.extractor_),
			orb_vocabulary_(frame.orb_vocabulary_),
			bow_vec_(frame.bow_vec_),
			feat_vector_(frame.feat_vector_),
			keypoints_num_(frame.keypoints_num_),
			levels_num_(frame.levels_num_)
		{
			for (int i = 0; i < FRAME_GRID_COLS; i++)
				for (int j = 0; j < FRAME_GRID_ROWS; j++)
					grid_[i][j] = frame.grid_[i][j];
			// 提供特征拷贝
			std::for_each(frame.features_.begin(), frame.features_.end(), [&](Feature* i)
			{Feature *tem = new Feature(*i); features_.clear(); features_.push_back(tem); });
		}

		Frame::~Frame()
		{
			std::for_each(features_.begin(), features_.end(), [&](Feature* i){delete i; });
		}

		cv::Mat Frame::getCameraCenter()
		{
			cv::Mat Rfw = Tcw_.rowRange(0, 3).colRange(0, 3);
			cv::Mat tfw = Tcw_.rowRange(0, 3).col(3);
			cv::Mat Ow = -Rfw.t()*tfw;
			return Ow;
		}

		std::vector<float> Frame::getScaleFactors()
		{
			std::vector<float> vec_scale_factor;
			vec_scale_factor.resize(levels_num_);
			vec_scale_factor[0] = 1.0f;
			for (int i = 1; i < levels_num_; i++)
			{
				vec_scale_factor[i] = vec_scale_factor[i - 1] * scale_factor_;
			}
			return vec_scale_factor;
		}

		void Frame::prepareImage(const cv::Mat& input_image, cv::Mat& gray_image)
		{
			gray_image = input_image;

			if (gray_image.channels() == 3)
			{
				if (is_rgb_order_)
					cvtColor(gray_image, gray_image, CV_RGB2GRAY);
				else
					cvtColor(gray_image, gray_image, CV_BGR2GRAY);
			}
			else if (gray_image.channels() == 4)
			{
				if (is_rgb_order_)
					cvtColor(gray_image, gray_image, CV_RGBA2GRAY);
				else
					cvtColor(gray_image, gray_image, CV_BGRA2GRAY);
			}
		}

		void Frame::extractORB(const cv::Mat &image)
		{
			std::vector<cv::KeyPoint> vec_keypoints;
			cv::Mat descriptors;
			(*extractor_)(image, cv::Mat(), vec_keypoints, descriptors);
			keypoints_num_ = vec_keypoints.size();
			if (keypoints_num_ == 0) return;

			features_.reserve(keypoints_num_);
			for (size_t i = 0; i < vec_keypoints.size(); i++)
			{
				// 这边内存释放放到析构中
				Feature *fea = new Feature(this, vec_keypoints[i], descriptors.row(i));
				features_.push_back(fea);
			}
		}


		bool Frame::isInFrame(const cv::Mat &point3d, cv::Point2f &point2d)
		{
			const cv::Mat Rcw = Tcw_.rowRange(0, 3).colRange(0, 3);
			const cv::Mat tcw = Tcw_.rowRange(0, 3).col(3);
			cv::Mat x3Dc = Rcw*point3d + tcw;
			const float xc = x3Dc.at<float>(0);
			const float yc = x3Dc.at<float>(1);
			const float invzc = 1.0 / x3Dc.at<float>(2);
			if (invzc < 0)
				return false;

			float u = cam_->fx()*xc*invzc + cam_->cx();
			float v = cam_->fy()*yc*invzc + cam_->cy();

			if (u<min_bound_x_ || u>max_bound_x_)
				return false;
			if (v<min_bound_y_ || v>max_bound_y_)
				return false;

			point2d.x = u;
			point2d.y = v;
		}

		void Frame::computeImageBounds(const cv::Mat &image)
		{
			// 如果是畸变图像
			if (cam_->distCoef().at<float>(0) != 0.0)
			{
				cv::Mat mat = (cv::Mat_<float>(4, 2) << 0.0f, 0.0f,
					image.cols, 0.0f,
					0.0f, image.rows,
					image.cols, image.rows);

				// 通过mat类型，转成4个点对即图像的4个边角点，进行畸变计算
				mat = mat.reshape(2);
				cv::undistortPoints(mat, mat, cam_->cvK(), cam_->distCoef(), cv::Mat(), cam_->cvK());
				mat = mat.reshape(1);
				// 对矫正之后的点选出最大最小边界值，也就是左上与左下进行比较获取x的最小值
				min_bound_x_ = std::min(mat.at<float>(0, 0), mat.at<float>(2, 0));
				max_bound_x_ = std::max(mat.at<float>(1, 0), mat.at<float>(3, 0));
				min_bound_y_ = std::min(mat.at<float>(0, 1), mat.at<float>(1, 1));
				max_bound_y_ = std::max(mat.at<float>(2, 1), mat.at<float>(3, 1));

			}
			else
			{
				min_bound_x_ = 0.0f;
				max_bound_x_ = image.cols;
				min_bound_y_ = 0.0f;
				max_bound_y_ = image.rows;
			}
		}

		bool Frame::posInGrid(const cv::KeyPoint &kp, int &pos_x, int &pos_y)
		{
			pos_x = round((kp.pt.x - min_bound_x_)*(1 / grid_element_height_));
			pos_y = round((kp.pt.y - min_bound_y_)*(1 / grid_element_width_));

			//由于特征点的坐标是进行了畸变矫正的，所以有可能在图像外边，这边要进行判断
			if (pos_x < 0 || pos_x >= FRAME_GRID_COLS || pos_y < 0 || pos_y >= FRAME_GRID_ROWS)
				return false;

			return true;
		}

		void Frame::assignFeaturesToGrid()
		{
			// 将特征分配到格子中
			int reserve_num = 0.5f*keypoints_num_ / (FRAME_GRID_COLS*FRAME_GRID_ROWS);
			for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
				for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
					grid_[i][j].reserve(reserve_num);

			for (int i = 0; i < keypoints_num_; i++)
			{
				const cv::KeyPoint &kp = features_[i]->undistored_keypoint_;

				int grid_pos_x, grid_pos_y;
				if (posInGrid(kp, grid_pos_x, grid_pos_y))
					grid_[grid_pos_x][grid_pos_y].push_back(i);
			}
		}

		std::vector<size_t> Frame::getFeaturesInArea(const float &x, const float  &y,
			const float  &r, const int min_level, const int max_level) const
		{
			std::vector<size_t> indices;
			indices.reserve(keypoints_num_);
			// 将特征点限制到格子的范围区间
			const int min_cell_x = std::max(0, (int)floor((x - min_bound_x_ - r)*(1 / grid_element_height_)));
			if (min_cell_x >= FRAME_GRID_COLS)
				return indices;

			const int max_cell_x = std::min((int)FRAME_GRID_COLS - 1, (int)ceil((x - min_bound_x_ + r)*(1 / grid_element_height_)));
			if (max_cell_x < 0)
				return indices;

			const int min_cell_y = std::max(0, (int)floor((y - min_bound_y_ - r)*(1 / grid_element_width_)));
			if (min_cell_y >= FRAME_GRID_ROWS)
				return indices;

			const int max_cell_y = std::min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - min_bound_y_ + r)*(1 / grid_element_width_)));
			if (max_cell_y < 0)
				return indices;

			const bool check_levels = (min_level>0) || (max_level >= 0);

			for (int ix = min_cell_x; ix <= max_cell_x; ix++)
			{
				for (int iy = min_cell_y; iy <= max_cell_y; iy++)
				{
					// 存储了特征点的索引
					const std::vector<size_t> cell = grid_[ix][iy];
					if (cell.empty())
						continue;

					for (size_t j = 0, jend = cell.size(); j < jend; j++)
					{
						const cv::KeyPoint &undistored_keypoint = features_[cell[j]]->undistored_keypoint_;
						// 再次对尺度进一步检测
						if (check_levels)
						{
							if (undistored_keypoint.octave < min_level)
								continue;
							if (max_level >= 0)
							{
								if (undistored_keypoint.octave > max_level)
									continue;
							}
						}
						// 再次确定特征点是否在半径r的范围内寻找的
						const float distx = undistored_keypoint.pt.x - x;
						const float disty = undistored_keypoint.pt.y - y;
						if (fabs(distx) < r && fabs(disty) < r)
							indices.push_back(cell[j]);
					}
				}
			}

			return indices;
		}

		std::vector<cv::Mat> Frame::toDescriptorVector(const Features &features)
		{
			std::vector<cv::Mat> vec_desc;
			int feature_size = features.size();
			vec_desc.reserve(feature_size);
			for (int j = 0; j < feature_size; j++)
				vec_desc.push_back(features[j]->descriptor_);

			return vec_desc;
		}

		void Frame::computeBoW()
		{
			if (bow_vec_.empty() || feat_vector_.empty())
			{
				std::vector<cv::Mat> current_desc = toDescriptorVector(features_);
				orb_vocabulary_->transform(current_desc, bow_vec_, feat_vector_, 4);
			}
		}

	}
}

