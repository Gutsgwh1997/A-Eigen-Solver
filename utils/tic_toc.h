#pragma once

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <string>

class TicToc {
public:
    TicToc() {
        tic();
    }

    void tic() {
        start = std::chrono::system_clock::now();
    }

    double toc() {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

//the following are UBUNTU/LINUX ONLY terminal color
const std::string RESET("\033[0m");
const std::string BLACK("\033[30m"); /* Black */
const std::string RED("\033[31m"); /* Red */
const std::string GREEN("\033[32m"); /* Green */
const std::string YELLOW("\033[33m"); /* Yellow */
const std::string BLUE("\033[34m"); /* Blue */
const std::string MAGENTA("\033[35m"); /* Magenta */
const std::string CYAN("\033[36m"); /* Cyan */
const std::string WHITE("\033[37m"); /* White */
const std::string BOLDBLACK("\033[1m\033[30m"); /* Bold Black */
const std::string BOLDRED("\033[1m\033[31m"); /* Bold Red */
const std::string BOLDGREEN("\033[1m\033[32m"); /* Bold Green */
const std::string BOLDYELLOW("\033[1m\033[33m"); /* Bold Yellow */
const std::string BOLDBLUE("\033[1m\033[34m"); /* Bold Blue */
const std::string BOLDMAGENTA("\033[1m\033[35m"); /* Bold Magenta */
const std::string BOLDCYAN("\033[1m\033[36m"); /* Bold Cyan */
const std::string BOLDWHITE("\033[1m\033[37m"); /* Bold White */
