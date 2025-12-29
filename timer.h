#ifndef TIMER_H
#define TIMER_H

#include <chrono>
#include <iostream>
#include <string>
#include <iomanip>

class Timer {
public:
    explicit Timer(const std::string& name = "")
        : name_(name), start_(std::chrono::high_resolution_clock::now()) {}

    ~Timer() {
        auto end = std::chrono::high_resolution_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_).count();
        std::cout << "[TIMER] " << name_ << " took " << std::fixed << std::setprecision(3) <<
            static_cast<float>(ms) / 1000 << " s" << std::endl;
    }

private:
    std::string name_;
    std::chrono::high_resolution_clock::time_point start_;
};

#endif // TIMER_H
