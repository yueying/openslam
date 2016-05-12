#include <openslam/utils/notify.h>
#include <windows.h>
#include <sstream>
#include <iostream>
#include <memory>

#define OPENSLAM_INIT_SINGLETON_PROXY(ProxyName, Func) static struct ProxyName{ ProxyName() { Func; } } s_##ProxyName;

namespace openslam
{
	namespace utils
	{
		/**定义空的流buffer*/
		class NullStreamBuffer : public std::streambuf
		{
		private:
			std::streamsize xsputn(const std::streambuf::char_type * /*str*/, std::streamsize n)
			{
				return n;
			}
		};

		/**定义空的输出流*/
		struct NullStream : public std::ostream
		{
		public:
			NullStream() :
				std::ostream(new NullStreamBuffer)
			{
				_buffer = dynamic_cast<NullStreamBuffer *>(rdbuf());
			}

			~NullStream()
			{
				rdbuf(0);
				delete _buffer;
			}

		protected:
			NullStreamBuffer* _buffer;
		};

		/** 当缓冲区需要同步时，流缓冲区调用通知处理程序(通常在 std::endl).
		*/
		struct NotifyStreamBuffer : public std::stringbuf
		{
			NotifyStreamBuffer() : _severity(openslam::utils::NOTICE)
			{
			}

			void setNotifyHandler(openslam::utils::NotifyHandler *handler) { _handler = std::shared_ptr<openslam::utils::NotifyHandler>(handler); }
			openslam::utils::NotifyHandler *getNotifyHandler() const { return _handler.get(); }

			/** 设置消息通知等级，给下一次通知使用 */
			void setCurrentSeverity(openslam::utils::NotifySeverity severity)
			{
				if (_severity != severity)
				{
					sync();
					_severity = severity;
				}
			}

			openslam::utils::NotifySeverity getCurrentSeverity() const { return _severity; }

		private:

			int sync()
			{
				sputc(0); // 字符串终止条件
				if (_handler)
					_handler->notify(_severity, pbase());
				pubseekpos(0, std::ios_base::out); // or str(std::string())
				return 0;
			}

			std::shared_ptr<openslam::utils::NotifyHandler> _handler;
			openslam::utils::NotifySeverity _severity;
		};

		struct NotifyStream : public std::ostream
		{
		public:
			NotifyStream() :
				std::ostream(new NotifyStreamBuffer)
			{
				_buffer = dynamic_cast<NotifyStreamBuffer *>(rdbuf());
			}

			void setCurrentSeverity(openslam::utils::NotifySeverity severity)
			{
				_buffer->setCurrentSeverity(severity);
			}

			openslam::utils::NotifySeverity getCurrentSeverity() const
			{
				return _buffer->getCurrentSeverity();
			}

			~NotifyStream()
			{
				rdbuf(0);
				delete _buffer;
			}

		protected:
			NotifyStreamBuffer* _buffer;
		};

	}
}

struct NotifySingleton
{
	NotifySingleton()
	{
		//默认消息等级
		_notifyLevel = openslam::utils::INFO;

		char* OPENSLAMNOTIFYLEVEL = getenv("OPENSLAM_NOTIFY_LEVEL");
		if (!OPENSLAMNOTIFYLEVEL) OPENSLAMNOTIFYLEVEL = getenv("OPENSLAMNOTIFYLEVEL");
		if (OPENSLAMNOTIFYLEVEL)
		{
			std::string stringOPENSLAMNOTIFYLEVEL(OPENSLAMNOTIFYLEVEL);

			// 转换为大写
			for (std::string::iterator i = stringOPENSLAMNOTIFYLEVEL.begin();
				i != stringOPENSLAMNOTIFYLEVEL.end();
				++i)
			{
				*i = toupper(*i);
			}

			if (stringOPENSLAMNOTIFYLEVEL.find("ALWAYS") != std::string::npos)          _notifyLevel = openslam::utils::ALWAYS;
			else if (stringOPENSLAMNOTIFYLEVEL.find("FATAL") != std::string::npos)      _notifyLevel = openslam::utils::FATAL;
			else if (stringOPENSLAMNOTIFYLEVEL.find("WARN") != std::string::npos)       _notifyLevel = openslam::utils::WARN;
			else if (stringOPENSLAMNOTIFYLEVEL.find("NOTICE") != std::string::npos)     _notifyLevel = openslam::utils::NOTICE;
			else if (stringOPENSLAMNOTIFYLEVEL.find("DEBUG_INFO") != std::string::npos) _notifyLevel = openslam::utils::DEBUG_INFO;
			else if (stringOPENSLAMNOTIFYLEVEL.find("DEBUG_FP") != std::string::npos)   _notifyLevel = openslam::utils::DEBUG_FP;
			else if (stringOPENSLAMNOTIFYLEVEL.find("DEBUG") != std::string::npos)      _notifyLevel = openslam::utils::DEBUG_INFO;
			else if (stringOPENSLAMNOTIFYLEVEL.find("INFO") != std::string::npos)       _notifyLevel = openslam::utils::INFO;
			else std::cout << "Warning: invalid OPENSLAM_NOTIFY_LEVEL set (" << stringOPENSLAMNOTIFYLEVEL << ")" << std::endl;

		}

		// 设置标准的消息通知handler
		openslam::utils::NotifyStreamBuffer *buffer = dynamic_cast<openslam::utils::NotifyStreamBuffer *>(_notifyStream.rdbuf());
		if (buffer && !buffer->getNotifyHandler())
			buffer->setNotifyHandler(new openslam::utils::StandardNotifyHandler);
	}

	openslam::utils::NotifySeverity _notifyLevel;
	openslam::utils::NullStream     _nullStream;
	openslam::utils::NotifyStream   _notifyStream;
};

static NotifySingleton& getNotifySingleton()
{
	static NotifySingleton s_NotifySingleton;
	return s_NotifySingleton;
}

bool openslam::utils::initNotifyLevel()
{
	getNotifySingleton();
	return true;
}

// 通过代理模式，强制NotifySingleton初始化
OPENSLAM_INIT_SINGLETON_PROXY(NotifySingletonProxy, openslam::utils::initNotifyLevel())

void openslam::utils::setNotifyLevel(openslam::utils::NotifySeverity severity)
{
	getNotifySingleton()._notifyLevel = severity;
}

openslam::utils::NotifySeverity openslam::utils::getNotifyLevel()
{
	return getNotifySingleton()._notifyLevel;
}

void openslam::utils::setNotifyHandler(openslam::utils::NotifyHandler *handler)
{
	openslam::utils::NotifyStreamBuffer *buffer = static_cast<openslam::utils::NotifyStreamBuffer*>(getNotifySingleton()._notifyStream.rdbuf());
	if (buffer) buffer->setNotifyHandler(handler);
}

openslam::utils::NotifyHandler* openslam::utils::getNotifyHandler()
{
	openslam::utils::NotifyStreamBuffer *buffer = static_cast<openslam::utils::NotifyStreamBuffer *>(getNotifySingleton()._notifyStream.rdbuf());
	return buffer ? buffer->getNotifyHandler() : 0;
}


#ifndef OPENSLAM_NOTIFY_DISABLED
bool openslam::utils::isNotifyEnabled(openslam::utils::NotifySeverity severity)
{
	return severity <= getNotifySingleton()._notifyLevel;
}
#endif

std::ostream& openslam::utils::notify(const openslam::utils::NotifySeverity severity)
{
	if (openslam::utils::isNotifyEnabled(severity))
	{
		getNotifySingleton()._notifyStream.setCurrentSeverity(severity);
		return getNotifySingleton()._notifyStream;
	}
	return getNotifySingleton()._nullStream;
}

void openslam::utils::StandardNotifyHandler::notify(openslam::utils::NotifySeverity severity, const char *message)
{
	if (severity <= openslam::utils::WARN)
		fputs(message, stderr);
	else
		fputs(message, stdout);
}

void openslam::utils::WinDebugNotifyHandler::notify(openslam::utils::NotifySeverity severity, const char *message)
{
	OutputDebugStringA(message);
}

