#pragma once

#include "nodes/NodeManager.h"
#include "nodes/subsystems/IDriveNode.h"
#include "api.h"
#include "nodes/actuator_nodes/MotorNode.h"
#include "nodes/sensor_nodes/ControllerNode.h"
#include "kinematics/TankDriveKinematics.h"

class TankDriveNode : public IDriveNode {
public: 
    struct TankEightMotors {
        MotorNode* left_1_motor;
        MotorNode* left_2_motor;
        MotorNode* left_3_motor;
        MotorNode* left_4_motor;
        MotorNode* right_1_motor;
        MotorNode* right_2_motor;
        MotorNode* right_3_motor;
        MotorNode* right_4_motor;
    };

    TankDriveNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, 
        TankEightMotors motors, TankDriveKinematics kinematics);

    void initialize();

    void resetEncoders();

    FourMotorDriveEncoderVals getIntegratedEncoderVals();

    void withDriveDirectionButtons(pros::controller_digital_e_t fwrd_btn,
            pros::controller_digital_e_t bwrd_btn);

    void setDriveVoltage(int y_voltage, int theta_voltage);

    void setDriveVoltage(int x_voltage, int y_voltage, int theta_voltage);

    void setDriveVelocity(float y_velocity, float theta_velocity);

    void setDriveVelocity(float x_velocity, float y_velocity, float theta_velocity);

    void setLeftVoltage(int voltage);

    void setRightVoltage(int voltage);

    void setLeftVelocity(float velocity);

    void setRightVelocity(float velocity);

    void teleopPeriodic();

    void autonPeriodic();

    ~TankDriveNode();

private:
    pros::Controller* m_controller;

    std::string m_handle_name;

    TankEightMotors m_motors;

    TankDriveKinematics m_kinematics;

    Eigen::Vector2d controller_target_velocity;

    pros::controller_digital_e_t m_fwrd_btn;

    pros::controller_digital_e_t m_bwrd_btn;

    bool reverse_direction;

    void m_setLeftPosition(float distance, int max_velocity);

    void m_setRightPosition(float distance, int max_velocity);
};
