#include "nodes/subsystems/IntakeNode.h"

IntakeNode::IntakeNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, 
pros::controller_digital_e_t intake_button, pros::controller_digital_e_t outtake_button, 
        std::vector<MotorNode*> intakes) : IRollerIntakeNode(node_manager, handle_name), 
        m_controller(controller->getController()),
        m_intakes(intakes),
        m_state(HOLDING),
        m_control_mode(ControlMode::INTAKE_OUTAKE_TOGGLE),
        m_btn_1(intake_button),
        m_btn_2(outtake_button) {
    m_handle_name = handle_name.insert(0, "robot/");
}

void IntakeNode::withControlMode(ControlMode control_mode) {
    m_control_mode = control_mode;
}

void IntakeNode::setIntakeVoltage(int voltage) {
    for (auto intake : m_intakes) {
        intake->moveVoltage(voltage);
    }
}

void IntakeNode::setIntakeVelocity(float velocity) {
    for (auto intake : m_intakes) {
        intake->moveVelocity(velocity);
    }
}

void IntakeNode::initialize() {

}

void IntakeNode::teleopPeriodic() {
    if (m_control_mode == ControlMode::INTAKE_OUTAKE_TOGGLE) {
        bool intakeButtonCurrentState = m_controller->get_digital(m_btn_1);
        bool outtakeButtonCurrentState = m_controller->get_digital(m_btn_2);

        switch (m_state) {
            case HOLDING:
                setIntakeVoltage(0);

                if (intakeButtonCurrentState && !m_previousIntakeBtnState) {
                    m_state = INTAKING;
                } else if (outtakeButtonCurrentState == 1) {
                    m_state = OUTTAKING;
                }

                break;
            case INTAKING:
                setIntakeVoltage(MAX_MOTOR_VOLTAGE);

                if (intakeButtonCurrentState && !m_previousIntakeBtnState) {
                    m_state = HOLDING;
                } else if (outtakeButtonCurrentState) {
                    m_state = OUTTAKING;
                }

                break;
            case OUTTAKING:
                setIntakeVoltage(-1 * MAX_MOTOR_VOLTAGE);

                if (intakeButtonCurrentState) {
                    m_state = INTAKING;
                } else if (!outtakeButtonCurrentState) {
                    m_state = HOLDING;
                }

                break;
            default:
                break;
        };

        // switch (m_control_mode) {
        // case INTAKE_OUTAKE_HOLD:
        //     switch (m_state) {
        //         case HOLDING:

        //     }
        //     break;
        // default:
        //     break;
        // }

        m_previousIntakeBtnState = intakeButtonCurrentState;
    } else {
        bool intakeButtonCurrentState = m_controller->get_digital(m_btn_1);
        bool dirBtnState = m_controller->get_digital(m_btn_2);

        if (intakeButtonCurrentState && !m_previousIntakeBtnState) {
            if (m_state == IntakeState::HOLDING) {
                m_state = IntakeState::INTAKING;
            } else {
                m_state = IntakeState::HOLDING;
            }
        }
        if (m_state == IntakeState::INTAKING) {
            setIntakeVoltage(MAX_MOTOR_VOLTAGE * (dirBtnState ? -1 : 1));
        } else {
            setIntakeVoltage(0);
        }

        m_previousIntakeBtnState = intakeButtonCurrentState;
    }
}

void IntakeNode::autonPeriodic() {
    
}

IntakeNode::~IntakeNode() {
    for (auto intake : m_intakes) {
        delete intake;
    }
}