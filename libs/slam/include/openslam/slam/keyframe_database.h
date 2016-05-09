#ifndef OPENSLAM_SLAM_KEYFRAME_DATABASE_H_
#define OPENSLAM_SLAM_KEYFRAME_DATABASE_H_
#include <openslam/slam/link_pragmas.h>
#include <mutex>
#include <openslam/slam/keyframe.h>
#include <openslam/slam/orb_vocabulary.h>

namespace openslam
{
	namespace slam
	{
		/** \brief 关键帧的数据库
		*/
		class SLAM_IMPEXP KeyFrameDatabase
		{
		public:

			KeyFrameDatabase(const ORBVocabulary &voc);

			void add(KeyFrame * keyframe);

			void erase(KeyFrame * keyframe);

			void clear();

		protected:

			const ORBVocabulary* orb_voc_;//!< 关联的词汇表

			std::vector<std::list<KeyFrame *> > inverted_file_;//!<词袋是逆序的

			std::mutex mutex_keyframe_;//!< 对关键帧更新的时候的互斥锁
		};
	}
}

#endif // OPENSLAM_SLAM_KEYFRAME_DATABASE_H_
