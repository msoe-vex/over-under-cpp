#pragma once

#include "nodes/NodeManager.h"
#include "api.h"
#include "nodes/actuator_nodes/MotorNode.h"
#include "nodes/sensor_nodes/ControllerNode.h"
#include "nodes/actuator_nodes/ADIDigitalOutNode.h"
#include "nodes/subsystems/IRollerIntakeNode.h"
#include "util/Constants.h"

class IntakeNode : public IRollerIntakeNode {
public:
    enum IntakeState {
        INTAKING, OUTTAKING, HOLDING
    };

    enum ControlMode {
        INTAKE_OUTAKE_TOGGLE,  
        TOGGLE_AND_SWITCH // One button toggles on/off and the other switches the direction with hold
    };

    IntakeNode(NodeManager* node_manager, std::string handle_name, 
    ControllerNode* controller, pros::controller_digital_e_t intake_button, 
    pros::controller_digital_e_t outtake_button, std::vector<MotorNode*> intakes);
    
    void withControlMode(ControlMode control_mode);

    void setIntakeVoltage(int voltage);

    void setIntakeVelocity(float velocity);

    void initialize();

    void teleopPeriodic();

    void autonPeriodic();

    ~IntakeNode();

private:
    IntakeState m_state;
    ControlMode m_control_mode;

    pros::Controller* m_controller;
    std::vector<MotorNode*> m_intakes;

    pros::controller_digital_e_t m_btn_1;
    pros::controller_digital_e_t m_btn_2;

    bool m_previousIntakeBtnState = false;

    std::string m_handle_name;
};
