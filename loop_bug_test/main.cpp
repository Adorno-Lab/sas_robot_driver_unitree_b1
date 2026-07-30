#include "dummy_driver.h"

    int main()
{

    std::cout << "\n===== TEST 1: shutdown() while callback is running =====\n";
    std::cout << "Starting loop...\n";
    auto t0 = std::chrono::steady_clock::now();
    {
        dummy_driver driver;
        driver.run_test_shutdown_while_running();
    }
    auto t1 = std::chrono::steady_clock::now();
    auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    std::cout
        << "shutdown() returned after " << elapsed_ms << " ms\n";

    {
      //  dummy_driver driver;
       // driver.run_test_shutdown_before_start();
    }

    std::cout << "\nmain() exiting\n";
    return 0;

    //shutdown() behaves as a blocking join on the callback thread, not an async signal.
}
