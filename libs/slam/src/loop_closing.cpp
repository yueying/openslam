#include <openslam/slam/loop_closing.h>

namespace openslam
{
	namespace slam
	{
		LoopClosing::LoopClosing(Map* map) :
			map_(map),
			loop_closing_stop_(false),
			thread_(nullptr),
			is_runing_(true)
		{

		}

		LoopClosing::~LoopClosing()
		{
			stopThread();
		}

		void LoopClosing::startThread()
		{
			thread_ = new std::thread(&LoopClosing::run, this);
		}

		void LoopClosing::stopThread()
		{
			if (thread_ != nullptr)
			{
				loop_closing_stop_ = true;
				thread_->detach();
				is_runing_ = false;
			}
		}

		void LoopClosing::run()
		{
			while (is_runing_)
			{

			}
			return ;
		}

		void LoopClosing::insertKeyFrame(KeyFrame * keyframe)
		{
			if (thread_ != nullptr)
			{
				std::unique_lock<mutex> lock(mutex_new_keyframes_);
				queue_new_keyframes_.push(keyframe);
			}
		}
	}
}