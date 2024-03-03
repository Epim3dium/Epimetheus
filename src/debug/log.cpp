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
std::unique_ptr<LogOutput> Log::out = std::make_unique<Output2Cerr>(Output2Cerr());

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

#ifdef __unix__ 
std::string Log::ToString(LogLevel level)
{
	static const char* const buffer[] = {"ERROR", "WARNING", "INFO", "DEBUG", "DEBUG1", "DEBUG2", "DEBUG3", "DEBUG4", "DEBUG5"};
    return buffer[level];
}
#else
std::string Log::ToString(LogLevel level)
{
	static const char* const buffer[] = {
        "\033[0;31mERROR\033[0m",
        "\033[0;33mWARNING\033[0m",
        "\033[1;37mINFO\033[0m",
        "\033[0;36mDEBUG\033[0m",
        "\033[0;36mDEBUG1\033[0m",
        "\033[0;36mDEBUG2\033[0m",
        "\033[0;36mDEBUG3\033[0m",
        "\033[0;36mDEBUG4\033[0m",
        "\033[0;36mDEBUG5\033[0m"};
    return buffer[level];
}
#endif


std::string NowTime() {
    return FormatTime(std::chrono::system_clock::now());
}

}
