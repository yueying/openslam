#include <opencv2/opencv.hpp>
#include <openslam/utils/cmd_line.h>
#include <openslam/utils/timer.h>
#include <openslam/slam/orb_extractor.h>

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
	std::vector<cv::KeyPoint> cv_feats;
	cv::Mat                   cv_descs;

	ORBextractor extractor;
	extractor(img, cv::Mat(), cv_feats, cv_descs);

	cv::Mat img_rgb = cv::Mat(img.size(), CV_8UC3);
	cv::cvtColor(img, img_rgb, CV_GRAY2RGB);
	std::for_each(cv_feats.begin(), cv_feats.end(), [&](cv::KeyPoint i){
		cv::circle(img_rgb, i.pt, 4 * (i.octave + 1), cv::Scalar(0, 255, 0), 1);
	});
	cv::imshow("orb_slam_img", img_rgb);

	cv::ORB orb;
	orb(img, cv::Mat(), cv_feats, cv_descs);

	cv::Mat img_opencv = cv::Mat(img.size(), CV_8UC3);
	cv::cvtColor(img, img_opencv, CV_GRAY2RGB);
	std::for_each(cv_feats.begin(), cv_feats.end(), [&](cv::KeyPoint i){
		cv::circle(img_opencv, i.pt, 4 * (i.octave + 1), cv::Scalar(0, 255, 0), 1);
	});
	cv::imshow("opencv_img", img_opencv);
	
	/// º∆À„ ±º‰
	Timer timer;
	for (int i = 0; i < 50; i++)
	{
		extractor(img, cv::Mat(), cv_feats, cv_descs);
	}
	double time = timer.Stop();
	std::cout << "opencv time " << time / 50.0 << std::endl;
	timer.Start();
	for (int i = 0; i < 50; i++)
	{
		orb(img, cv::Mat(), cv_feats, cv_descs);
	}
	time = timer.Stop();
	std::cout << "orb-slam time " << time / 50.0 << std::endl;
	cv::waitKey(0);
	return 0;
}