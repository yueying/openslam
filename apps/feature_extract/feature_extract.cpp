#include <opencv2/opencv.hpp>
#include <openslam/utils/cmd_line.h>
#include <openslam/utils/timer.h>
#include <openslam/slam/orb_extractor.h>
#include <openslam/slam/frame.h>
#include <openslam/slam/feature.h>
#include <openslam/slam/monocular_camera.h>

using namespace openslam::utils;
using namespace openslam::slam;

int main(int argc, char *argv[])
{
	CmdLine cmd;
	std::string img_name;
	cmd.add(make_option('i', img_name, "imgname"));
	try {
		if (argc == 1) throw std::string("Invalid command line parameter.");
		cmd.process(argc, argv);
	}
	catch (const std::string& s) {
		std::cerr << "Feature detector \nUsage: " << argv[0] << "\n"
			<< "[-i|--imgname name]\n"
			<< std::endl;

		std::cerr << s << std::endl;
		return EXIT_FAILURE;
	}
	cv::Mat img(cv::imread(img_name, 0));
	assert(img.type() == CV_8UC1 && !img.empty());
	

	AbstractCamera *cam = new MonocularCamera(640.0, 480.0,
		517.306408, 516.469215, 318.643040, 255.313989,
		0.262383, -0.953104, -0.005358, 0.002628, 1.163314);
	ORBextractor *extractor = new ORBextractor(2000);
	Timer timer;
	Frame frame(cam, img, 0, extractor);
	double time = timer.Stop();
	std::cout << "Original time " << time<< std::endl;
	std::cout << "frame keypoint num " << frame.getKeypointsNum() << std::endl;
	cv::Mat img_rgb(img.size(), CV_8UC3);
	cv::cvtColor(img, img_rgb, CV_GRAY2RGB);
	std::vector<Feature *> cv_feats = frame.features_;
	std::for_each(cv_feats.begin(), cv_feats.end(), [&](Feature *i){
		cv::circle(img_rgb, i->keypoint_.pt, 4 * (i->keypoint_.octave + 1), cv::Scalar(0, 255, 0), 1);
		cv::circle(img_rgb, i->undistored_keypoint_.pt, 4 * (i->undistored_keypoint_.octave + 1), cv::Scalar(0, 0, 255), 1);
	});
	cv::imshow("Original", img_rgb);

	// 测试先对图像进行畸变矫正后进行特征提取
	cv::Mat un_img;
	std::vector<cv::KeyPoint> cv_keypoints;
	cv::Mat                   cv_descs;
	timer.Start();
	cam->undistortImage(img, un_img);
	(*extractor)(un_img, cv::Mat(), cv_keypoints, cv_descs);
	time = timer.Stop();
	std::cout << "Undistort frame time " << time << std::endl;
	std::cout << "Undistort frame keypoint num " << cv_keypoints.size() << std::endl;
	cv::Mat img_opencv(img.size(), CV_8UC3);
	cv::cvtColor(un_img, img_opencv, CV_GRAY2RGB);
	std::for_each(cv_keypoints.begin(), cv_keypoints.end(), [&](cv::KeyPoint i){
		cv::circle(img_opencv, i.pt, 4 * (i.octave + 1), cv::Scalar(0, 255, 0), 1);
	});
	cv::imshow("Undistort", img_opencv);
	cv::waitKey(0);

	delete cam;
	delete extractor;
	getchar();
	return 0;
}