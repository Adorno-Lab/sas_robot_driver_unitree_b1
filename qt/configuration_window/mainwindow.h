/*
# (C) Copyright 2024-2026 Adorno-Lab software developments
#
#    This file is part of sas_robot_driver_unitree_b1.
#
#    This is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This software is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with this software.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana, email: juanjose.quirozomana@manchester.ac.uk
#
# ################################################################*/

#pragma once
#include <QMainWindow>
//#include "DriverUnitreeB1.hpp"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    //void on_horizontalSlider_forward_speed_sliderMoved(int position);

    void update_horizontal_slider_roll();
    void update_horizontal_slider_pitch();
    void update_horizontal_slider_yaw();
    void update_vertical_slider_height();


    void update_horizontal_slider_forward_speed();
    void update_horizontal_slider_side_speed();
    void update_horizontal_slider_yaw_speed();

private:
    Ui::MainWindow *ui;
    void timerEvent(QTimerEvent *event);
    int timerId_;
    int time_step_in_milliseconds_;
    double elapsed_time_;

    double target_roll_{0};
    double target_pitch_{0};
    double target_yaw_{0};
    double target_height_{0};

    double target_forward_speed_{0};
    double target_side_speed_{0};
    double target_yaw_speed_{0};

    const double slider_factor_ = 100.0;

    void _set_zero_commands();

    void _connect();
    void _initialize();
    void _deinitialize();
    void _disconnect();




};
