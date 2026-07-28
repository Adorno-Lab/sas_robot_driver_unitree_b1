#include "dummy_driver.h"
#include <atomic>
#include <iostream>


using namespace std;


/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>

static std::atomic_bool kill_this_process(false);

void sig_int_handler(int);

void sig_int_handler(int)
{
    kill_this_process = true;
}


int main()
{
    /*
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("::Error setting the signal int handler.");
    }
    dummy_driver driver(&kill_this_process, 0.002);

    driver.connect();
    driver.initialize();
    int iterations = 20;
    for (int i=0;i<= iterations;i++ )
    {
        std::cout<<driver.get_motion_time()<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    driver.deinitialize();
    driver.disconnect();

    return 0;
*/
    // We don't even need the full driver for this test
    {
        // Create a LoopFunc that does nothing quickly
        auto cb = [](){ std::this_thread::sleep_for(std::chrono::milliseconds(1)); };
        auto loop = std::make_shared<UNITREE_LEGGED_SDK::LoopFunc>("test", 0.001, cb);

        loop->start();

        // Let it run for a short time
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Shutdown is non-blocking (0 ms, as you saw)
        auto start = std::chrono::steady_clock::now();
        loop->shutdown();
        auto end = std::chrono::steady_clock::now();
        std::cout << "shutdown() took "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                  << " ms" << std::endl;

        // Now destroy the shared_ptr (this will call ~Loop())
        std::cout << "Resetting loop... (watch for crash or delay)" << std::endl;
        start = std::chrono::steady_clock::now();
        loop.reset();   // triggers destructor
        end = std::chrono::steady_clock::now();

        std::cout << "~Loop() took "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                  << " ms" << std::endl;
    }
    std::cout << "Program finished safely." << std::endl;
    return 0;
}
