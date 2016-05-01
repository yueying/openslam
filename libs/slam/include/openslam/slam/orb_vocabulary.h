#ifndef OPENSLAM_SLAM_ORB_VOCABULARY_H_
#define OPENSLAM_SLAM_ORB_VOCABULARY_H_

#include<DBoW2/FORB.h>
#include<DBoW2/TemplatedVocabulary.h>

namespace openslam
{
	namespace slam
	{
		typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
			ORBVocabulary;
	}
}

#endif // OPENSLAM_SLAM_ORB_VOCABULARY_H_
