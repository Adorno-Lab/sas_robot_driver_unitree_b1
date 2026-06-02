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

    void update_horizontalSlider_roll();
    void update_horizontalSlider_pitch();
    void update_horizontalSlider_yaw();
    void update_verticalSlider_height();

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

    void _set_zero_commands();
};
