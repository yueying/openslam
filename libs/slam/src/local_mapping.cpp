#include <openslam/slam/local_mapping.h>

namespace openslam
{
	namespace slam
	{
		LocalMapping::LocalMapping(Map* map):
			map_(map),
			local_mapping_stop_(false),
			thread_(nullptr),
			is_runing_(true)
		{

		}

		LocalMapping::~LocalMapping()
		{
			stopThread();
		}

		void LocalMapping::startThread()
		{
			thread_ = new std::thread(&LocalMapping::run,this);
		}

		void LocalMapping::stopThread()
		{
			if (thread_ != nullptr)
			{
				local_mapping_stop_ = true;
				thread_->detach();
				is_runing_ = false;
			}
		}

		void LocalMapping::run()
		{
			while (is_runing_)
			{

			}
			return ;
		}

		void LocalMapping::insertKeyFrame(KeyFrame * keyframe)
		{
			if (thread_ != nullptr)
			{
				std::unique_lock<mutex> lock(mutex_new_keyframes_);
				list_new_keyframes_.push(keyframe);
			}
		}
	}
}