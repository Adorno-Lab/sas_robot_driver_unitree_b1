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
    timerId_ = startTimer(time_step_in_milliseconds_);

    ui->horizontalSlider_roll_->setRange(-30,30);
    ui->horizontalSlider_roll_->setTickInterval(1);
    ui->doubleSpinBox_roll_->setRange(-0.3,0.3);

    ui->horizontalSlider_pitch_->setRange(-30,30);
    ui->horizontalSlider_pitch_->setTickInterval(1);
    ui->doubleSpinBox_pitch_->setRange(-0.3,0.3);

    ui->horizontalSlider_yaw_->setRange(-60,60);
    ui->horizontalSlider_yaw_->setTickInterval(1);
    ui->doubleSpinBox_yaw_->setRange(-0.6,0.6);

    ui->verticalSlider_height_delta_->setRange(-16,16);
    ui->verticalSlider_height_delta_->setTickInterval(1);
    ui->doubleSpinBox_height_delta_->setRange(-0.16,0.16);

    connect(ui->pushButton_zero_commands_, &QPushButton::clicked,
            this,&MainWindow::_set_zero_commands);

    connect(ui->horizontalSlider_roll_, &QSlider::sliderMoved,
            this, &MainWindow::update_horizontalSlider_roll);
    connect(ui->horizontalSlider_pitch_, &QSlider::sliderMoved,
            this, &MainWindow::update_horizontalSlider_pitch);
    connect(ui->horizontalSlider_yaw_, &QSlider::sliderMoved,
            this, &MainWindow::update_horizontalSlider_yaw);
    connect(ui->verticalSlider_height_delta_, &QSlider::sliderMoved,
            this, &MainWindow::update_verticalSlider_height);
}

MainWindow::~MainWindow()
{
    delete ui;
    killTimer(timerId_);
}

void MainWindow::update_horizontalSlider_roll()
{
    target_roll_  = ui->horizontalSlider_roll_->sliderPosition()/100.0;
    ui->doubleSpinBox_roll_->setValue(target_roll_);

}

void MainWindow::update_horizontalSlider_pitch()
{
    target_pitch_ = ui->horizontalSlider_pitch_->sliderPosition()/100.0;
    ui->doubleSpinBox_pitch_->setValue(target_pitch_);
}

void MainWindow::update_horizontalSlider_yaw()
{
    target_yaw_ = ui->horizontalSlider_yaw_->sliderPosition()/100.0;
    ui->doubleSpinBox_yaw_->setValue(target_yaw_);
}

void MainWindow::update_verticalSlider_height()
{
    target_height_ = ui->verticalSlider_height_delta_->sliderPosition()/100.0;
    ui->doubleSpinBox_height_delta_->setValue(target_height_);
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

    qDebug() << "target_roll: " << target_roll_ << " target_pitch: "<<target_pitch_<<" target_yaw: "<<target_yaw_;
    ui->elapsed_time_label_->setText(time_string);

    //update_horizontalSlider_roll();
}

void MainWindow::_set_zero_commands()
{
    target_roll_ = 0;
    target_pitch_ = 0;
    target_yaw_ = 0;
    target_height_ = 0;

    ui->horizontalSlider_roll_->setSliderPosition(0);
    ui->doubleSpinBox_roll_->setValue(0);
    ui->horizontalSlider_pitch_->setSliderPosition(0);
    ui->doubleSpinBox_pitch_->setValue(0);
    ui->horizontalSlider_yaw_->setSliderPosition(0);
    ui->doubleSpinBox_yaw_->setValue(0);
    ui->verticalSlider_height_delta_->setSliderPosition(0);
    ui->doubleSpinBox_height_delta_->setValue(0);

}



