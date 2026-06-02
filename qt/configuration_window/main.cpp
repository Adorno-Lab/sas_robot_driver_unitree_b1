#include "mainwindow.h"

#include <QApplication>

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


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w{&kill_this_process};
    w.show();
    return a.exec();
}
