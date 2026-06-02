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



    ui->horizontalSlider_forward_speed_->setRange(-10,10);
    ui->horizontalSlider_forward_speed_->setTickInterval(1);
    ui->doubleSpinBox_forward_speed_->setRange(-0.1,0.1);

    ui->horizontalSlider_side_speed_->setRange(-10,10);
    ui->horizontalSlider_side_speed_->setTickInterval(1);
    ui->doubleSpinBox_side_speed_->setRange(-0.1,0.1);

    ui->horizontalSlider_yaw_->setRange(-30,30);
    ui->horizontalSlider_yaw_->setTickInterval(1);
    ui->doubleSpinBox_yaw_->setRange(-0.3,0.3);


    ui->dial_select_robot_->setRange(1,3);
    ui->dial_select_robot_->setValue(2);
    //ui->dial_select_robot_->set


    ui->pushButton_connect_->setEnabled(false);
    ui->pushButton_initialize_->setEnabled(false);
    ui->pushButton_deinitialize_->setEnabled(false);
    ui->pushButton_disconnect_->setEnabled(false);

    ui->pushButton_zero_commands_->setStyleSheet(pause_yellow_color_);

    connect(ui->pushButton_zero_commands_, &QPushButton::clicked,
            this,&MainWindow::_set_zero_commands);

    connect(ui->horizontalSlider_roll_, &QSlider::sliderMoved,
            this, &MainWindow::update_horizontal_slider_roll);
    connect(ui->horizontalSlider_pitch_, &QSlider::sliderMoved,
            this, &MainWindow::update_horizontal_slider_pitch);
    connect(ui->horizontalSlider_yaw_, &QSlider::sliderMoved,
            this, &MainWindow::update_horizontal_slider_yaw);
    connect(ui->verticalSlider_height_delta_, &QSlider::sliderMoved,
            this, &MainWindow::update_vertical_slider_height);


    connect(ui->horizontalSlider_forward_speed_, &QSlider::sliderMoved,
            this, &MainWindow::update_horizontal_slider_forward_speed);
    connect(ui->horizontalSlider_side_speed_, &QSlider::sliderMoved,
            this, &MainWindow::update_horizontal_slider_side_speed);
    connect(ui->horizontalSlider_yaw_speed_, &QSlider::sliderMoved,
            this, &MainWindow::update_horizontal_slider_yaw_speed);


    connect(ui->pushButton_connect_, &QPushButton::clicked,
            this, &MainWindow::_connect);
    connect(ui->pushButton_initialize_, &QPushButton::clicked,
            this, &MainWindow::_initialize);
    connect(ui->pushButton_deinitialize_, &QPushButton::clicked,
            this, &MainWindow::_deinitialize);
    connect(ui->pushButton_disconnect_, &QPushButton::clicked,
            this, &MainWindow::_disconnect);

    connect(ui->dial_select_robot_, &QDial::valueChanged,
            this, &MainWindow::update_dial_select_robot);

}

MainWindow::~MainWindow()
{
    delete ui;
    killTimer(timerId_);
}

void MainWindow::update_horizontal_slider_roll()
{
    target_roll_  = ui->horizontalSlider_roll_->sliderPosition()/slider_factor_;
    ui->doubleSpinBox_roll_->setValue(target_roll_);

}

void MainWindow::update_horizontal_slider_pitch()
{
    target_pitch_ = ui->horizontalSlider_pitch_->sliderPosition()/slider_factor_;
    ui->doubleSpinBox_pitch_->setValue(target_pitch_);
}

void MainWindow::update_horizontal_slider_yaw()
{
    target_yaw_ = ui->horizontalSlider_yaw_->sliderPosition()/slider_factor_;
    ui->doubleSpinBox_yaw_->setValue(target_yaw_);
}

void MainWindow::update_vertical_slider_height()
{
    target_height_ = ui->verticalSlider_height_delta_->sliderPosition()/slider_factor_;
    ui->doubleSpinBox_height_delta_->setValue(target_height_);
}

void MainWindow::update_horizontal_slider_forward_speed()
{
    target_forward_speed_ = ui->horizontalSlider_forward_speed_->sliderPosition()/slider_factor_;
    ui->doubleSpinBox_forward_speed_->setValue(target_forward_speed_);
}

void MainWindow::update_horizontal_slider_side_speed()
{
    target_side_speed_ = ui->horizontalSlider_side_speed_->sliderPosition()/slider_factor_;
    ui->doubleSpinBox_side_speed_->setValue(target_side_speed_);
}

void MainWindow::update_horizontal_slider_yaw_speed()
{
    target_yaw_speed_ = ui->horizontalSlider_yaw_speed_->sliderPosition()/slider_factor_;
    ui->doubleSpinBox_yaw_speed_->setValue(target_yaw_speed_);
}

void MainWindow::update_dial_select_robot()
{
    int sr = ui->dial_select_robot_->sliderPosition();

    std::string ip = "192.168.8.170";

    if (sr == 1) //White robot
    {
        configuration_.ROBOT_IP = "192.168.8.170";
        configuration_.LIE_DOWN_ROBOT_WHEN_DEINITIALIZE = false;
        configuration_.ROBOT_PORT = 8082;
        configuration_.FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO = false;
        ui->label_ip_->setText(QString(("IP: "+configuration_.ROBOT_IP).c_str()));
        ui->pushButton_connect_->setEnabled(true);
    }else if (sr == 2)
    {
        ui->pushButton_connect_->setEnabled(false);
    }else if (sr == 3) //Black robot
    {
        configuration_.ROBOT_IP = "192.168.8.226";
        configuration_.LIE_DOWN_ROBOT_WHEN_DEINITIALIZE = false;
        configuration_.ROBOT_PORT = 8082;
        configuration_.FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO = true;
        ui->label_ip_->setText(QString(("IP: "+configuration_.ROBOT_IP).c_str()));
        ui->pushButton_connect_->setEnabled(true);
    }



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
    qDebug() << "vx: " << target_forward_speed_ << " vy: "<<target_side_speed_<<" vw: "<<target_yaw_speed_;
    ui->elapsed_time_label_->setText(time_string);

    //update_horizontalSlider_roll();
}

void MainWindow::_set_zero_commands()
{
    target_roll_ = 0;
    target_pitch_ = 0;
    target_yaw_ = 0;
    target_height_ = 0;

    target_forward_speed_ = 0;
    target_side_speed_ = 0;
    target_yaw_speed_ = 0;

    ui->horizontalSlider_roll_->setSliderPosition(0);
    ui->doubleSpinBox_roll_->setValue(0);

    ui->horizontalSlider_pitch_->setSliderPosition(0);
    ui->doubleSpinBox_pitch_->setValue(0);

    ui->horizontalSlider_yaw_->setSliderPosition(0);
    ui->doubleSpinBox_yaw_->setValue(0);

    ui->verticalSlider_height_delta_->setSliderPosition(0);
    ui->doubleSpinBox_height_delta_->setValue(0);


    ui->horizontalSlider_forward_speed_->setSliderPosition(0);
    ui->doubleSpinBox_forward_speed_->setValue(0);

    ui->horizontalSlider_side_speed_->setSliderPosition(0);
    ui->doubleSpinBox_side_speed_->setValue(0);

    ui->horizontalSlider_yaw_speed_->setSliderPosition(0);
    ui->doubleSpinBox_yaw_speed_->setValue(0);

}

void MainWindow::_connect()
{
    ui->pushButton_connect_->setEnabled(false);
    ui->pushButton_initialize_->setEnabled(true);
    ui->dial_select_robot_->setEnabled(false);
}

void MainWindow::_initialize()
{
    ui->pushButton_initialize_->setEnabled(false);
    ui->pushButton_deinitialize_->setEnabled(true);
}

void MainWindow::_deinitialize()
{
    ui->pushButton_deinitialize_->setEnabled(false);
    ui->pushButton_disconnect_->setEnabled(true);
}

void MainWindow::_disconnect()
{

    ui->pushButton_disconnect_->setEnabled(false);
}





