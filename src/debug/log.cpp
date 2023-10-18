#include "log.hpp"
namespace epi {

std::string FormatTime(std::chrono::system_clock::time_point tp) {
    std::stringstream ss;
    auto t = std::chrono::system_clock::to_time_t(tp);
    auto tp2 = std::chrono::system_clock::from_time_t(t);
    if (tp2 > tp)
        t = std::chrono::system_clock::to_time_t(tp - std::chrono::seconds(1));
    ss  << std::put_time(std::localtime(&t), "%Y-%m-%d %T")
        << "." << std::setfill('0') << std::setw(3)
        << (std::chrono::duration_cast<std::chrono::microseconds>(
           tp.time_since_epoch()).count() % 1000000) ;
    return ss.str();
}

LogLevel Log::ReportingLevel = DEBUG3;
LogOutput* Log::out = new Output2Cerr();

Log::Log()
{
}

std::ostringstream& Log::Get(LogLevel level)
{
    os << "- " << NowTime();
    os << " [" << ToString(level) << "]: ";
    os << std::string(level > DEBUG ? level - DEBUG : 0, '\t');
    return os;
}

Log::~Log()
{
    os << std::endl;
    out->Output(os.str());
}

std::string Log::ToString(LogLevel level)
{
	static const char* const buffer[] = {"ERROR", "WARNING", "INFO", "DEBUG", "DEBUG1", "DEBUG2", "DEBUG3", "DEBUG4", "DEBUG5"};
    return buffer[level];
}


std::string NowTime() {
    return FormatTime(std::chrono::system_clock::now());
}

}
