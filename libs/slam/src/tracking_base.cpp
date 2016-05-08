#include <openslam/slam/tracking_base.h>

namespace openslam
{
	namespace slam
	{
		TrackingBase::TrackingBase(ORBVocabulary *voc):
			orb_vocabulary_(voc),
			tracking_state_(TRACKING_NO_IMAGES_YET),
			is_only_tracking_(false),
			is_vo_(false)
		{
			//´´½¨µØÍ¼
			map_ = new Map();
		}
		TrackingBase::~TrackingBase()
		{
			delete map_;
		}
	}
}