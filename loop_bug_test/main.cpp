#include "dummy_driver.h"

    int main()
{
    {
        dummy_driver driver;
        driver.run_test_shutdown_while_running();
    }

    {
        dummy_driver driver;
        driver.run_test_shutdown_before_start();
    }

    std::cout << "\nmain() exiting\n";
    return 0;

    //shutdown() behaves as a blocking join on the callback thread, not an async signal.
}
