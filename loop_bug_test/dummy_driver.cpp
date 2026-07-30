#include "dummy_driver.h"

dummy_driver::dummy_driver()
{
    make_loop();
}

dummy_driver::~dummy_driver()
{
    std::cout << "\n========== dummy_driver destructor ==========\n";
}

void dummy_driver::make_loop()
{
    loop_ = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>(
        "test_loop",
        0.001,
        boost::bind(&dummy_driver::callback, this));
}

void dummy_driver::callback()
{
    callback_started_ = true;
    std::cout << "Callback entered\n";

    //
    // Simulate a callback that takes a long time.
    //
    std::this_thread::sleep_for(std::chrono::seconds(4));

    //
    // Access object memory AFTER the sleep.
    //
    counter_++;
    std::cout
        << "Callback resumed after sleep\n"
        << "counter = " << counter_ << "\n";
    callback_finished_ = true;
    std::cout << "Callback finished\n";
}

void dummy_driver::run_test_shutdown_while_running()
{

    loop_->start();

    //
    // Wait until callback has entered.
    //
    while (!callback_started_)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    std::cout << "\nMain thread detected callback is running.\n";

    //loop_.reset();
    //std::cout << "Calling shutdown()...\n";
    //loop_->shutdown();


//std::cout << "\nDestroying LoopFunc immediately...\n";

   // std::cout << "LoopFunc destroyed.\n";

   // std::cout
    //    << "callback_finished = " << callback_finished_ << "\n";
/*
    if (elapsed_ms > 5000)
        std::cout << "[RESULT] shutdown() BLOCKED for the callback duration -> join-like behavior.\n";
    else
        std::cout << "[RESULT] shutdown() returned quickly despite callback still sleeping -> detach-like behavior.\n";
*/
    //loop_->shutdown();
}

void dummy_driver::run_test_shutdown_before_start()
{
    std::cout << "\n===== TEST 2: shutdown() called before callback ever starts =====\n";

    // Fresh state for this test.
    callback_started_ = false;
    callback_finished_ = false;
    counter_ = 0;
    make_loop();

    std::cout << "Starting loop...\n";
    loop_->start();

    // Immediately shut down, without waiting for callback_started_.
    // If the LoopFunc thread hasn't even entered the callback yet
    // (or the SDK internally waits for at least one period tick),
    // this exercises a different code path than Test 1.
    auto t0 = std::chrono::steady_clock::now();
    std::cout << "Calling shutdown() immediately (no wait)...\n";
    loop_->shutdown();
    auto t1 = std::chrono::steady_clock::now();

    auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    std::cout
        << "shutdown() returned after " << elapsed_ms << " ms\n";
    std::cout
        << "callback_started_  = " << callback_started_ << "\n"
        << "callback_finished_ = " << callback_finished_ << "\n";

    std::cout << "\nDestroying LoopFunc immediately...\n";
    loop_.reset();
    std::cout << "LoopFunc destroyed.\n";

    if (elapsed_ms < 2000)
        std::cout << "[RESULT] shutdown() returned quickly when callback wasn't mid-run -> not a fixed-duration wait.\n";
    else
        std::cout << "[RESULT] shutdown() took a while even though callback hadn't started -> unexpected, investigate.\n";
}
