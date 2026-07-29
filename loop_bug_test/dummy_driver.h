#pragma once
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <unitree_legged_sdk/unitree_legged_sdk.h>

class dummy_driver
{
public:
    dummy_driver();
    ~dummy_driver();

    // Test 1: shutdown() called *while* the callback is mid-execution
    // (sleeping). Confirms whether shutdown() blocks (join-like) until
    // the callback returns, or returns immediately (detach-like).
    void run_test_shutdown_while_running();

    // Test 2: shutdown() called *before* the callback has ever started.
    // Confirms shutdown() doesn't always block for some fixed/long
    // duration regardless of callback state -- i.e. it's not just
    // "always waits ~10s no matter what".
    void run_test_shutdown_before_start();

private:
    std::atomic<bool> callback_started_{false};
    std::atomic<bool> callback_finished_{false};
    unsigned long long counter_{0};
    std::shared_ptr<UNITREE_LEGGED_SDK::LoopFunc> loop_;

    void callback();
    void make_loop(); // (re)create loop_ with a fresh LoopFunc
};
