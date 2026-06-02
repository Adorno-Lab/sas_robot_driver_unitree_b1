#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow),
    time_step_in_milliseconds_{100},
    elapsed_time_{0}
{
    ui->setupUi(this);
    setWindowTitle("Unitree B1 driver test");
    ui->statusbar->showMessage("Welcome!", 5000);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::timerEvent([[maybe_unused]] QTimerEvent *event)
{
    elapsed_time_ += time_step_in_milliseconds_;

    // Convert to seconds
    int total_seconds = static_cast<int>(elapsed_time_ / 1000.0);
    // Calculate hours, minutes, seconds
    int hours = total_seconds / 3600;
    int minutes = (total_seconds % 3600) / 60;
    int seconds = total_seconds % 60;

    // Format as hh:mm:ss
    QString time_string = QString("%1:%2:%3")
                              .arg(hours, 2, 10, QChar('0'))
                              .arg(minutes, 2, 10, QChar('0'))
                              .arg(seconds, 2, 10, QChar('0'));

    //qDebug() << "Elapsed time: " << time_string;
    //ui->elapsed_time_label->setText(time_string);
}
