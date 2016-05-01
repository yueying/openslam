#include <openslam/slam/tracking_base.h>

namespace openslam
{
	namespace slam
	{
		TrackingBase::TrackingBase(ORBVocabulary *voc):
			orb_vocabulary_(voc)
		{

		}
		TrackingBase::~TrackingBase()
		{

		}
	}
}