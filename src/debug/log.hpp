#ifndef EPI_LOG_H
#define EPI_LOG_H
#include <fstream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <chrono>
#include <iomanip>
#include <iostream>

namespace epi {


std::string NowTime();

//enum TLogLevel {logERROR, logWARNING, logINFO, logDEBUG, logDEBUG1, logDEBUG2, logDEBUG3, logDEBUG4};
enum LogLevel {
    ERROR,
    WARNING,
    INFO,
    DEBUG,
    DEBUG1,
    DEBUG2,
    DEBUG3
};

class LogOutput {
public:
    virtual void Output(std::string msg) = 0;
};

class Log
{
public:
    Log();
    virtual ~Log();
    std::ostringstream& Get(LogLevel level = INFO);
public:
    static LogOutput* out;
    static LogLevel ReportingLevel;
    static std::string ToString(LogLevel level);
protected:
    std::ostringstream os;
private:
    Log(const Log&);
    Log& operator =(const Log&);
};

class Output2Cerr : public  LogOutput {
public:
    void Output(std::string msg) override {
        std::cerr << msg;
    }
};

class Output2FILE : public  LogOutput {
    std::ofstream file;
public:
    void Output(std::string msg) override {
        file << msg;
    }
    Output2FILE(std::string filename) : file(filename) {}
};

#ifndef FILELOG_MAX_LEVEL
#define FILELOG_MAX_LEVEL DEBUG3
#endif

#define EPI_LOG(level) \
    if (level > FILELOG_MAX_LEVEL) ;\
    else if (level > Log::ReportingLevel) ; \
    else Log().Get(level)

//#define L_(level) \
//if (level > FILELOG_MAX_LEVEL) ;\
//else if (level > FILELog::ReportingLevel || !Output2FILE::Stream()) ; \
//else FILELog().Get(level)



}
#endif //EPI_LOG_H
