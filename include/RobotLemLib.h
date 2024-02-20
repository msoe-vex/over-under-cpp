#pragma once

#include "IRobot.h"

class RobotLemLib : public IRobot
{
public:
    /* Node Manager */
    NodeManager *node_manager;

    lemlib::Chassis* chassis;

    /* Everything is an intake because we are so good at planning :) */
    // MotorNode *intake_motor;
    // IntakeNode *intake_node;

    // MotorNode *indexer_motor;
    // IntakeNode *indexer_node;

    // MotorNode *roller_motor;
    // IntakeNode *roller_node;

    // MotorNode *extra_motor_1;
    // IntakeNode *extra_node_1;

    // MotorNode *extra_motor_2;
    // IntakeNode *extra_node_2;

    void initialize() override;
    void disabled() override;
    void competition_initialize() override;
    void autonomous() override;
    void opcontrol() override;

private:
    void readConfig();
    void screenPrinting();
    int readPort(std::ifstream &file);
    bool readReversed(std::ifstream &file);

    std::vector<std::pair<int, bool>> configVector;
    std::string print_string;
};
