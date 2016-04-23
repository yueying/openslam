/*******************************************************************************
* 文件： notify.h
* 时间： 2016/4/10 15:55
* 作者： 冯兵
* 邮件： fengbing123@gmail.com
*
* 说明： 主要用来做日志处理
*
********************************************************************************/
#ifndef OPENSLAM_UTILS_NOTIFY_H_
#define OPENSLAM_UTILS_NOTIFY_H_

#include <openslam/base/link_pragmas.h>
#include <ostream>

namespace openslam
{
	namespace utils
	{
		/** 提示通知的严重程度
		*/
		enum NotifySeverity {
			ALWAYS = 0,
			FATAL = 1,
			WARN = 2,
			NOTICE = 3,
			INFO = 4,
			DEBUG_INFO = 5,
			DEBUG_FP = 6
		};

		/// 用于判断是否开启通知
#ifdef OPENSLAM_NOTIFY_DISABLED
		inline bool isNotifyEnabled(NotifySeverity) { return false; }
#else
		/** 用于设置大于当前通知级别的才进行通知 */
		extern BASE_IMPEXP bool isNotifyEnabled(NotifySeverity severity);
#endif

		/** 设置通知的输出等级，通过设置环境变量来处理 OPENSLAMNOTIFYLEVEL 或者 OPENSLAM_NOTIFY_LEVEL.
		*/
		extern BASE_IMPEXP void setNotifyLevel(NotifySeverity severity);

		/** 得到当前通知的输出等级 */
		extern BASE_IMPEXP NotifySeverity getNotifyLevel();

		/** 初始化通知等级 */
		extern BASE_IMPEXP bool initNotifyLevel();

		/** 通过设置NotifyLevel来进行对输出消息进行控制，可以设置环境变量，如：
		* - 设置 OPENSLAMNOTIFYLEVEL=DEBUG
		* 默认等级为NOTICE
		*
		* \code
		* openslam::utils::notify(openslam::utils::DEBUG) << "Hello Bugs!" << std::endl;
		* \endcode
		*/
		extern BASE_IMPEXP std::ostream& notify(const NotifySeverity severity);

		/**默认通知输出*/
		inline std::ostream& notify(void) { return notify(openslam::utils::INFO); }

#define OPENSLAM_NOTIFY(level) if (openslam::utils::isNotifyEnabled(level)) openslam::utils::notify(level)
#define OPENSLAM_ALWAYS OPENSLAM_NOTIFY(openslam::utils::ALWAYS)
#define OPENSLAM_FATAL OPENSLAM_NOTIFY(openslam::utils::FATAL)
#define OPENSLAM_WARN OPENSLAM_NOTIFY(openslam::utils::WARN)
#define OPENSLAM_NOTICE OPENSLAM_NOTIFY(openslam::utils::NOTICE)
#define OPENSLAM_INFO OPENSLAM_NOTIFY(openslam::utils::INFO)
#define OPENSLAM_DEBUG OPENSLAM_NOTIFY(openslam::utils::DEBUG_INFO)
#define OPENSLAM_DEBUG_FP OPENSLAM_NOTIFY(openslam::utils::DEBUG_FP)

		/** Handler 处理消息流的输出. 被作为一个输出槽的功能，当消息需要同步时
		* (如调用 openslam::utils::notify() << std::endl).
		* StandardNotifyHandler被默认使用，其将消息输出到stderr
		* (severity <= WARN) 或者 stdout (severity > WARN).
		* 消息可以被重定向到，其它GUI的窗体上或者windows debugger (WinDebugNotifyHandler).
		* 使用setNotifyHandle来自定义handler.
		* 注意：消息输出 API不是线程安全的，输出的GUI的时候要进行线程控制
		* \see setNotifyHandler
		*/
		class BASE_IMPEXP NotifyHandler
		{
		public:
			virtual ~NotifyHandler(){}
			virtual void notify(openslam::utils::NotifySeverity severity, const char *message) = 0;
		};

		/** 设置消息handler, 默认是StandardNotifyHandler
		* \see NotifyHandler
		*/
		extern BASE_IMPEXP void setNotifyHandler(NotifyHandler *handler);

		/** 得到当前的消息通知handler. */
		extern BASE_IMPEXP NotifyHandler *getNotifyHandler();

		/** 将消息通知流重定向到stderr (severity <= WARN) 或者 stdout (severity > WARN).
		* fputs()函数用于将消息写入到标准文件中，则标准流
		* std::out and std::cerr streams将不会被使用.
		* \see setNotifyHandler
		*/
		class BASE_IMPEXP StandardNotifyHandler : public NotifyHandler
		{
		public:
			void notify(openslam::utils::NotifySeverity severity, const char *message);
		};

		/** 将消息通知流重定向到 windows debugger 通过使用 OuputDebugString 函数.
		* \see setNotifyHandler
		*/
		class BASE_IMPEXP WinDebugNotifyHandler : public NotifyHandler
		{
		public:
			void notify(openslam::utils::NotifySeverity severity, const char *message);
		};
	}

}



#endif // OPENSLAM_UTILS_NOTIFY_H_
