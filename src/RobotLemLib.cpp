#include "RobotLemLib.h"

void RobotLemLib::readConfig()
{
    std::ifstream file;
    file.open("/usd/robotLemLib.txt");

    if (file.is_open())
    {
        for (int i = 0; i < 13; ++i)
        {
            configVector.push_back(std::make_pair(readPort(file), readReversed(file)));
        }
    }
    else
    {
        primary_controller->updateDisplay(pros::E_CONTROLLER_MASTER, "Error reading config");
    }
}

int RobotLemLib::readPort(std::ifstream &file)
{
    std::string portString = "";
    std::getline(file, portString);
    return stoi(portString);
}

bool RobotLemLib::readReversed(std::ifstream &file) {
    std::string reversedString = "";
    std::getline(file, reversedString);
    int reversedInt = stoi(reversedString);
    return reversedInt == 1;
}

void RobotLemLib::screenPrinting() {
    int i = 0;
    while (true) {
        lemlib::Pose pose = chassis->getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::lcd::print(3, "%d", i); // print the heading
        // primary_controller->updateDisplay(pros::E_CONTROLLER_MASTER, "x: " + std::to_string(pose.x));
        i++;
        pros::delay(10);
    }
}

// Initializes 18in robot
void RobotLemLib::initialize()
{
    pros::lcd::initialize();
    node_manager = new NodeManager(pros::millis);
    primary_controller = new ControllerNode(node_manager, "primary");
    primary_controller->updateDisplay(pros::E_CONTROLLER_MASTER, "Running LemLib");

    readConfig();

    /* Define the drivetrain components*/
    // pros::Motor left_drive_1(configVector[0].first, pros::E_MOTOR_GEARSET_18, configVector[0].second); // Port 1, Gearset, Reverse-False
    // pros::Motor left_drive_2(configVector[1].first, pros::E_MOTOR_GEARSET_18, configVector[1].second); // Port 1, Gearset, Reverse-False
    // pros::Motor left_drive_3(configVector[2].first, pros::E_MOTOR_GEARSET_18, configVector[2].second); // Port 1, Gearset, Reverse-False
    // pros::Motor left_drive_4(configVector[3].first, pros::E_MOTOR_GEARSET_18, configVector[3].second); // Port 1, Gearset, Reverse-False

    // pros::Motor right_drive_1(configVector[4].first, pros::E_MOTOR_GEARSET_18, configVector[4].second); // Port 1, Gearset, Reverse-False
    // pros::Motor right_drive_2(configVector[5].first, pros::E_MOTOR_GEARSET_18, configVector[5].second); // Port 1, Gearset, Reverse-False
    // pros::Motor right_drive_3(configVector[6].first, pros::E_MOTOR_GEARSET_18, configVector[6].second); // Port 1, Gearset, Reverse-False
    // pros::Motor right_drive_4(configVector[7].first, pros::E_MOTOR_GEARSET_18, configVector[7].second); // Port 1, Gearset, Reverse-False

    pros::Motor left_drive_1(0, pros::E_MOTOR_GEARSET_18, false); // Port 1, Gearset, Reverse-False
    pros::Motor left_drive_2(1, pros::E_MOTOR_GEARSET_18, false); // Port 1, Gearset, Reverse-False
    pros::Motor left_drive_3(2, pros::E_MOTOR_GEARSET_18, false); // Port 1, Gearset, Reverse-False
    pros::Motor left_drive_4(3, pros::E_MOTOR_GEARSET_18, false); // Port 1, Gearset, Reverse-False

    pros::Motor right_drive_1(4, pros::E_MOTOR_GEARSET_18, false); // Port 1, Gearset, Reverse-False
    pros::Motor right_drive_2(5, pros::E_MOTOR_GEARSET_18, false); // Port 1, Gearset, Reverse-False
    pros::Motor right_drive_3(6, pros::E_MOTOR_GEARSET_18, false); // Port 1, Gearset, Reverse-False
    pros::Motor right_drive_4(7, pros::E_MOTOR_GEARSET_18, false); // Port 1, Gearset, Reverse-False

    pros::MotorGroup left_side_motors({left_drive_1, left_drive_2, left_drive_3, left_drive_4});
    pros::MotorGroup right_side_motors({right_drive_1, right_drive_2, right_drive_3, right_drive_4});

    lemlib::Drivetrain_t drivetrain {
        &left_side_motors, // left drivetrain motors
        &right_side_motors, // right drivetrain motors
        10.75, // track width (in?) TODO: Change this
        3.25, // wheel diameter (in?)
        200 // wheel rpm 200 RPM carts direct to the wheels
    };

    /* Define Odometry Components */ //TODO Address this Odometry section
    
    pros::ADIEncoder x_enc('D', 'E', true); // ports A and B, reversed
    pros::ADIEncoder y_enc('B', 'C', false); // ports C and D, not reversed
    pros::Rotation other_enc('H');
    
    // left tracking wheel TODO: Change these values
    lemlib::TrackingWheel x_tracking_wheel(&x_enc, 2.0, -3.75); // 2.75" wheel diameter, -4.6" offset from tracking center
    // right tracking wheel TODO: Change these values
    lemlib::TrackingWheel y_tracking_wheel(&y_enc, 2.0, -2.0); // 2.75" wheel diameter, 4.5" offset from tracking center
    lemlib::TrackingWheel other_tracking_wheel(&other_enc, 2.0, 2.0);
    
    // inertial sensor
    pros::Imu inertial_sensor(16); // port 2 TODO: Change this port
    
    // odometry struct
    lemlib::OdomSensors_t sensors {
        &x_tracking_wheel, // vertical tracking wheel 1
        &other_tracking_wheel,
        &y_tracking_wheel, // horizontal tracking wheel 1
        nullptr, // we don't have a second tracking wheel, so we set it to nullptr
        &inertial_sensor // inertial sensor
    };

    // lemlib::OdomSensors_t sensors {
    //     nullptr, // vertical tracking wheel 1
    //     nullptr,
    //     nullptr, // horizontal tracking wheel 1
    //     nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    //     nullptr // inertial sensor
    // };

    /* Define PID Components */
    
    // Forward/Backward PID
    lemlib::ChassisController_t lateralController {
        8, // kP
        30, // kD
        1, // smallErrorRange
        100, // smallErrorTimeout
        3, // largeErrorRange
        500, // largeErrorTimeout
        5 // slew rate
    };

    // Turning PID
    lemlib::ChassisController_t angularController {
        4, // kP
        40, // kD
        1, // smallErrorRange
        100, // smallErrorTimeout
        3, // largeErrorRange
        500, // largeErrorTimeout
        0 // slew rate
    };

    // Create Chassis
    chassis = new lemlib::Chassis(drivetrain, lateralController, angularController, sensors);

    chassis->calibrate();
    // pros::Task screenPrintingTask(RobotLemLib::screenPrinting);
    RobotLemLib::screenPrinting();
    
    /* Define the intake components */
    // intake_motor = new MotorNode(node_manager, configVector[8].first, "intake", configVector[8].second);
    // indexer_motor = new MotorNode(node_manager, configVector[9].first, "indexer", configVector[9].second);

    // intake_node = new IntakeNode(
    //     node_manager, "intake", primary_controller,
    //     pros::E_CONTROLLER_DIGITAL_A, pros::E_CONTROLLER_DIGITAL_B, {intake_motor});

    // indexer_node = new IntakeNode(
    //     node_manager, "indexer", primary_controller,
    //     pros::E_CONTROLLER_DIGITAL_A, pros::E_CONTROLLER_DIGITAL_B, {indexer_motor});

    // /* Define the Roller Components */
    // roller_motor = new MotorNode(node_manager, configVector[10].first, "roller", configVector[10].second);

    // roller_node = new IntakeNode(
    //     node_manager, "roller", primary_controller,
    //     pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_L2, {roller_motor});
    
    // extra_motor_1 = new MotorNode(node_manager, configVector[11].first, "extra1", configVector[11].second);

    // extra_node_1 = new IntakeNode(
    //     node_manager, "extra1", primary_controller,
    //     pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_L2, {extra_motor_1});
        
    // extra_motor_2 = new MotorNode(node_manager, configVector[12].first, "extra2", configVector[12].second);

    // extra_node_2 = new IntakeNode(
    //     node_manager, "extra2", primary_controller,
    //     pros::E_CONTROLLER_DIGITAL_X, pros::E_CONTROLLER_DIGITAL_Y, {extra_motor_2});
}

void RobotLemLib::disabled() {}

void RobotLemLib::competition_initialize() {}

void RobotLemLib::autonomous() {}

// Must put all telepPeriodic() method from each class into here
void RobotLemLib::opcontrol()
{
    while (true)
    {
        // nodeManager->executeTeleop();
        // tank_drive_node->teleopPeriodic();
        // intake_node->teleopPeriodic();
        // indexer_node->teleopPeriodic();
        // roller_node->teleopPeriodic();
        // extra_node_1->teleopPeriodic();
        // extra_node_2->teleopPeriodic();
        // RobotLemLib::screenPrinting();
    }
}
