#pragma once
#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <numeric>

namespace epi {
    class timer {
        //utils for strcutured time measurement
        public:
        typedef  std::chrono::duration<double> duration_t;

        static std::map<std::string, duration_t>& TimeTable() {
            static std::map<std::string, duration_t> s_time_table;
            return s_time_table;
        };

        static std::vector<std::string>& CurGroup() {
            static std::vector<std::string> s_group_vec;
            return s_group_vec;
        }
        static std::string getIdGroup() {
            std::string s;
            for(auto& g : CurGroup())
                s += g + ".";
            return s;
        }
        class stopper {
            std::string id;
            bool m_isgroup;
            std::string group;
            std::chrono::time_point<std::chrono::steady_clock> m_start;
            std::chrono::time_point<std::chrono::steady_clock> m_stop;
            duration_t m_dur;
        public:
            void start() {
                m_start = std::chrono::steady_clock::now();
            }
            void stop() {
                auto stop = std::chrono::steady_clock::now();
                m_dur += stop - m_start;
            }
            stopper(std::string ID, bool isGroup = false) {
                m_isgroup = isGroup;
                group = ID;
                id = getIdGroup() + ID;
                if(isGroup)
                    CurGroup().push_back(group);
            }
            ~stopper() {
                TimeTable()[id] += m_dur;
                if(m_isgroup)
                    for(int i = 0; i < CurGroup().size(); i++)
                        if(CurGroup()[i] == group) {
                            CurGroup().erase(CurGroup().begin() + i);
                            break;
                        }
            }

        };

        //______________
        //use scope when it will not the majority of tested thing's time (not for tight loops)
        class scope {
            std::string id;
            bool m_isgroup;
            std::string group;
            std::chrono::time_point<std::chrono::steady_clock> m_start;

        public:
            scope(std::string ID, bool isGroup = false) : m_isgroup(isGroup) {
                m_start = std::chrono::steady_clock::now();
                group = ID;
                id = getIdGroup() + ID;
                if(isGroup)
                    CurGroup().push_back(group);
            }
            ~scope() {
                auto stop = std::chrono::steady_clock::now();
                TimeTable()[id] += stop - m_start;
                if(m_isgroup)
                    for(int i = 0; i < CurGroup().size(); i++)
                        if(CurGroup()[i] == group) {
                            CurGroup().erase(CurGroup().begin() + i);
                            break;
                        }
            }
        };
        
        //______________
        class Get {
            std::string m_id;
            std::string m_group;
            static void displayGroup(std::string id, std::ostream& os, std::string preCout = "") {
                duration_t sum = TimeTable()[id];
                for(auto t : TimeTable()) {
                    if(t.first.find(id) == 0 && t.first.size() != id.size()) {

                        //checking if its in not the current id group
                        std::string str = t.first;
                        str = t.first.substr(id.size() + 1);
                        if(str.find(".") != std::string::npos)
                            continue;

                        os << preCout << t.first << "\t" << t.second / sum * 100.f << "%\n";
                        displayGroup(t.first, os, preCout + "\t");
                    }
                }
            }
        public:
            double sec() {
                return std::chrono::duration_cast<std::chrono::seconds>(TimeTable()[m_id]).count();
            }
            double ms() {
                return std::chrono::duration_cast<std::chrono::milliseconds>(TimeTable()[m_id]).count();
            }
            double um() {
                return std::chrono::duration_cast<std::chrono::microseconds>(TimeTable()[m_id]).count();
            }
            void displayGroup(std::ostream& os) {
                os << "total of \"" << m_id << "\" " << ms() << " ms\n";
                displayGroup(m_id, os, "\t");
            }
            void reset() {
                TimeTable().erase(m_id);
            }
            Get(std::string id) : m_id(id) { }
        };

    };
    static std::ostream& operator<< (std::ostream& stream, timer::Get getter) {
        getter.displayGroup(stream);
        return stream;
    }


};
