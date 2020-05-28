#ifndef LOGGER_H
#define LOGGER_H

#ifdef _WIN32
#ifdef CELEX_API_EXPORTS
#define CELEX_EXPORTS __declspec(dllexport)
#else
#define CELEX_EXPORTS __declspec(dllimport)
#endif
#else
#if defined(LOGGER_LIBRARY)
#define CELEX_EXPORTS
#else
#define CELEX_EXPORTS
#endif
#endif

#define GLOG_NO_ABBREVIATED_SEVERITIES

#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <mutex>
#include <glog/logging.h>

#define LOG_INFO(ModuleName) \
  google::LogMessage(__FILE__, __LINE__, google::GLOG_INFO).stream() \
      << "[" << ModuleName << "]"

#define LOG_WARN(ModuleName) \
  google::LogMessage(__FILE__, __LINE__, google::GLOG_WARNING).stream() \
      << "[" << ModuleName << "]"

#define LOG_ERROR(ModuleName) \
  google::LogMessage(__FILE__, __LINE__, google::GLOG_ERROR).stream() \
      << "[" << ModuleName << "]"

#define LOG_FATAL(ModuleName) \
  google::LogMessage(__FILE__, __LINE__, google::GLOG_FATAL).stream() \
      << "[" << ModuleName << "]"

void CELEX_EXPORTS InitLogging(std::string dir);

void CELEX_EXPORTS ShutdownLogging();

class Logger : public google::base::Logger
{
public:
	Logger(std::string dir) : log_dir_(dir){}
	void Write(bool force_flush,
		time_t timestamp,
		const char* message,
		int message_len);
	void Flush() {}
	google::uint32 LogSize() { return msg_data_.length(); }

private:
	std::map<std::string, std::ofstream*> mModuleToStreamMap;
	std::string msg_data_;
	std::string log_dir_;
	std::mutex mutex_;
};

#endif //LOGGER_H