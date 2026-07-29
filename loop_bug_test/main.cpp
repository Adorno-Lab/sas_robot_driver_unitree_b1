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
}
