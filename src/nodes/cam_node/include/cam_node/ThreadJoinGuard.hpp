#pragma once

#include <thread>
#include <exception>
#include <chrono>
#include <utility>

class ThreadJoinGuard
{
public:
    explicit ThreadJoinGuard(std::thread &thread)
        : thread_(thread)
    {
    }

    ~ThreadJoinGuard()
    {
        if (thread_.joinable())
        {
            thread_.join();
        }
    }

    ThreadJoinGuard(const ThreadJoinGuard &) = delete;
    ThreadJoinGuard &operator=(const ThreadJoinGuard &) = delete;

private:
    std::thread &thread_;
};
