#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <cmath>

MainWindow::MainWindow(std::atomic_bool *break_loops, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow),
    st_break_loops_{break_loops},
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

    ui->horizontalSlider_yaw_speed_->setRange(-10,10);
    ui->horizontalSlider_yaw_speed_->setTickInterval(1);
    ui->doubleSpinBox_yaw_speed_->setRange(-0.1,0.1);


    ui->dial_select_robot_->setRange(1,3);
    ui->dial_select_robot_->setValue(2);

    ui->dial_change_operation_high_level_mode_->setRange(0,1);
    ui->dial_change_operation_high_level_mode_->setValue(0);

    //ui->doubleSpinBox_read_vx_->setRange(-5,5);
    //ui->doubleSpinBox_read_vy_->setRange(-5,5);
    //ui->doubleSpinBox_read_wz_->setRange(-5,5);

    _config_spin_boxes_as_read_only({ui->doubleSpinBox_read_vx_,
                                     ui->doubleSpinBox_read_vy_,
                                     ui->doubleSpinBox_read_wz_,
                                     ui->doubleSpinBox_read_roll_,
                                     ui->doubleSpinBox_read_pitch_,
                                     ui->doubleSpinBox_read_yaw_,
                                     ui->doubleSpinBox_read_height_,
                                     ui->doubleSpinBox_computed_roll_,
                                     ui->doubleSpinBox_computed_pitch_,
                                     ui->doubleSpinBox_computed_yaw_,
                                     ui->doubleSpinBox_computed_rel_roll_,
                                     ui->doubleSpinBox_computed_rel_pitch_,
                                     ui->doubleSpinBox_computed_rel_yaw_
                                     });


    ui->pushButton_connect_->setEnabled(false);
    ui->pushButton_initialize_->setEnabled(false);
    ui->pushButton_deinitialize_->setEnabled(false);
    ui->pushButton_disconnect_->setEnabled(false);

    ui->pushButton_zero_commands_->setStyleSheet(pause_yellow_color_);

    // The driver has no target mode: it walks with the commanded speeds (see DriverUnitreeB1::_robot_control()).
    ui->pushButton_target_mode_->setText("N/A");
    ui->pushButton_target_mode_->setEnabled(false);

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
    connect(ui->dial_change_operation_high_level_mode_, &QDial::valueChanged,
            this, &MainWindow::update_dial_change_operation_high_level_mode);

}

MainWindow::~MainWindow()
{
    if (timerId_ != 0)
        killTimer(timerId_);
    // _deinitialize() and _disconnect() use ui, so it is deleted afterwards.
    _deinitialize();
    _disconnect();
    delete ui;
    std::cerr<<"destructor called"<<std::endl;
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
        configuration_.FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO = true; //true
        ui->label_ip_->setText(QString(("IP: "+configuration_.ROBOT_IP).c_str()));
        ui->pushButton_connect_->setEnabled(true);
    }



}

void MainWindow::update_dial_change_operation_high_level_mode()
{
    /*
    if (unitree_b1_driver_)
    {
        int dial = ui->dial_change_operation_high_level_mode_->sliderPosition();
        if (dial == 0) //walking
            unitree_b1_driver_->request_change_in_high_level_control(DriverUnitreeB1::HIGH_LEVEL_MODE::TARGET_VELOCITY_WALKING);
        if (dial == 1)
            unitree_b1_driver_->request_change_in_high_level_control(DriverUnitreeB1::HIGH_LEVEL_MODE::FORCED_STAND);
    }
  */
}

void MainWindow::timerEvent([[maybe_unused]] QTimerEvent *event)
{
    // Set by the SIGINT handler. Closing the window deinitializes the driver in the destructor.
    if (*st_break_loops_)
    {
        killTimer(timerId_);
        timerId_ = 0;
        close();
        return;
    }

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

  //  qDebug() << "target_roll: " << target_roll_ << " target_pitch: "<<target_pitch_<<" target_yaw: "<<target_yaw_;
  //  qDebug() << "vx: " << target_forward_speed_ << " vy: "<<target_side_speed_<<" vw: "<<target_yaw_speed_;
    ui->elapsed_time_label_->setText(time_string);

    //update_horizontalSlider_roll();
    if (unitree_b1_driver_)
    {
        std::string status_msg = unitree_b1_driver_->get_status_message();
        ui->label_status_->setText(QString(status_msg.c_str()));

        unitree_b1_driver_->set_high_level_speed(target_forward_speed_, target_side_speed_, target_yaw_speed_);
        //unitree_b1_driver_->set_forced_stand_commands(target_roll_, target_pitch_, target_yaw_, target_height_);

        VectorXd w = unitree_b1_driver_->get_high_level_angular_velocity().vec3();
        VectorXd v   = unitree_b1_driver_->get_high_level_linear_velocity().vec3();



        if (w(2) > max_read_wz_)
            max_read_wz_ = w(2);
        if (v(0) > max_read_vx_)
            max_read_vx_ = v(0);
        if (v(1) > max_read_vy_)
            max_read_vy_ = v(1);


            max_filtered_read_vy_ = v(1);



        // Euler angles computed from the IMU orientation, and relative to the orientation recorded while stopped
        const DQ r_0b = unitree_b1_driver_->get_IMU_orientation();
        if (is_unit(r_0b))
        {
            const Eigen::Vector3d rpy_from_orientation = _compute_euler_angles_from_unit_quaternion(r_0b);
            ui->doubleSpinBox_computed_roll_->setValue(rpy_from_orientation.x());
            ui->doubleSpinBox_computed_pitch_->setValue(rpy_from_orientation.y());
            ui->doubleSpinBox_computed_yaw_->setValue(rpy_from_orientation.z());

            const DQ r_0f = unitree_b1_driver_->get_last_IMU_orientation_when_robot_stopped();
            if (is_unit(r_0f))
            {
                const Eigen::Vector3d rpy_rel = _compute_euler_angles_from_unit_quaternion(r_0f.conj()*r_0b);
                ui->doubleSpinBox_computed_rel_roll_->setValue(rpy_rel.x());
                ui->doubleSpinBox_computed_rel_pitch_->setValue(rpy_rel.y());
                ui->doubleSpinBox_computed_rel_yaw_->setValue(rpy_rel.z());
            }
        }

        // Euler angles reported by the IMU
        const Vector3d rpy = unitree_b1_driver_->get_IMU_rpy_angles();
        ui->doubleSpinBox_read_roll_->setValue(rpy.x());
        ui->doubleSpinBox_read_pitch_->setValue(rpy.y());
        ui->doubleSpinBox_read_yaw_->setValue(rpy.z());

        ui->doubleSpinBox_read_vx_->setValue(v(0));
        ui->doubleSpinBox_read_vy_->setValue(v(1));
        ui->doubleSpinBox_read_wz_->setValue(w(2));

        ui->progressBar_battery->setValue(unitree_b1_driver_->get_state_of_charge());

        const std::string current_mode = DriverUnitreeB1::high_level_mode_to_string(
                                         unitree_b1_driver_->get_current_high_mode());
        const std::string gait_type = DriverUnitreeB1::gait_type_to_string(
                                      unitree_b1_driver_->get_current_gait_type());
        ui->pushButton_current_mode_->setText(QString(current_mode.c_str()));
        ui->pushButton_gait_type_->setText(QString(gait_type.c_str()));

        ui->doubleSpinBox_read_height_->setValue(unitree_b1_driver_->get_body_height());








    }
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
    ui->dial_select_robot_->setEnabled(false);

    if (!unitree_b1_driver_)
    {
        std::vector<DriverUnitreeB1::CUSTOM_FLAGS> custom_flags;
        if (configuration_.FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO)
            custom_flags.push_back(DriverUnitreeB1::CUSTOM_FLAGS::FORCE_STAND_MODE_WHEN_HIGH_LEVEL_VELOCITIES_ARE_ZERO);
        unitree_b1_driver_ = std::make_shared<DriverUnitreeB1>(st_break_loops_,
                                          DriverUnitreeB1::MODE::VelocityControl, // Driver mode
                                          DriverUnitreeB1::LEVEL::HIGH,       // Level mode
                                          true,   //verbosity
                                          2000,   // TIMEOUT in ms
                                          configuration_.LIE_DOWN_ROBOT_WHEN_DEINITIALIZE, // LIE DOWN ROBOT WHEN DEINITIALIZE
                                          configuration_.ROBOT_IP,  // Target IP   //192.168.123.10 for low-level mode
                                          configuration_.ROBOT_PORT,// Target port  //8007 for low-level mode
                                          8090,
                                          custom_flags);

    }
    try
    {
        unitree_b1_driver_->connect();
        ui->pushButton_initialize_->setEnabled(true);
    }
    catch (const std::exception& e)
    {
        // Release the driver (and its UDP port), so that the connection can be tried again.
        unitree_b1_driver_.reset();
        std::cerr<<"MainWindow::_connect. "<<e.what()<<std::endl;
        ui->statusbar->showMessage(QString("Connection failed: ") + e.what(), 10000);
        ui->label_status_->setText("Connection failed!");
        ui->pushButton_connect_->setEnabled(true);
        ui->dial_select_robot_->setEnabled(true);
    }
}

void MainWindow::_initialize()
{
    ui->pushButton_initialize_->setEnabled(false);
    ui->pushButton_deinitialize_->setEnabled(true);
    if (unitree_b1_driver_)
    {
        unitree_b1_driver_->initialize();
    }
}

void MainWindow::_deinitialize()
{
    ui->pushButton_deinitialize_->setEnabled(false);
    ui->pushButton_disconnect_->setEnabled(true);
    if (unitree_b1_driver_)
        unitree_b1_driver_->deinitialize();
}

void MainWindow::_disconnect()
{

    ui->pushButton_disconnect_->setEnabled(false);
    if (unitree_b1_driver_)
        unitree_b1_driver_->disconnect();
}

void MainWindow::_config_spin_boxes_as_read_only(const std::vector<QDoubleSpinBox *> &spinboxes)
{
    for (size_t i = 0; i < spinboxes.size(); ++i) {
        spinboxes.at(i)->setReadOnly(true);
        spinboxes.at(i)->setButtonSymbols(QAbstractSpinBox::NoButtons);
        spinboxes.at(i)->setAlignment(Qt::AlignCenter); // Optional: center align
        spinboxes.at(i)->setDecimals(2);  // Set precision to 3 decimals
        spinboxes.at(i)->setRange(-500,500);
    }
}

Vector3d MainWindow::_compute_euler_angles_from_unit_quaternion(const DQ &r) const
{
    // The columns of the rotation matrix are the rotated axes.
    const VectorXd x_axis = (r*i_*r.conj()).vec3();
    const VectorXd y_axis = (r*j_*r.conj()).vec3();
    const VectorXd z_axis = (r*k_*r.conj()).vec3();

    // ZYX (roll-pitch-yaw) angles
    double roll_manual  = std::atan2(y_axis(2), z_axis(2));
    double pitch_manual = std::atan2(-x_axis(2),
                                     std::sqrt(y_axis(2)*y_axis(2) + z_axis(2)*z_axis(2)));
    double yaw_manual   = std::atan2(x_axis(1), x_axis(0));


    // Return as Vector3d (roll, pitch, yaw) - radians
    Vector3d euler_angles;
    euler_angles << roll_manual, pitch_manual, yaw_manual;

    return euler_angles;
}



