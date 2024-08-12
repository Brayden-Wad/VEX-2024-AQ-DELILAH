#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "lemlib/api.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

pros::Controller primaryController(pros::E_CONTROLLER_MASTER);
pros::Controller secondaryController(pros::E_CONTROLLER_PARTNER);

pros::Motor frontLeftDrive(11, pros::v5::MotorGears::green);
pros::Motor middleLeftDrive(12, pros::v5::MotorGears::green);
pros::Motor rearLeftDrive(13, pros::v5::MotorGears::green);
pros::Motor frontRightDrive(-20, pros::v5::MotorGears::green);
pros::Motor middleRightDrive(-19, pros::v5::MotorGears::green);
pros::Motor rearRightDrive(-18, pros::v5::MotorGears::green);
pros::Motor test(1, pros::v5::MotorGears::green);
pros::Motor intakeLower(14, pros::v5::MotorGears::green);
pros::Motor intakeUpper(15, pros::v5::MotorGears::green);
pros::Motor arm(16, pros::v5::MotorGears::green);

pros::Imu imu(17);

pros::adi::Encoder encoderVertical('A', 'B', false);
pros::adi::Encoder encoderHorizontal('C', 'D', true);

pros::MotorGroup drivetrainLeft({-11, -12, -13});
pros::MotorGroup drivetrainRight({20, 19, 18});

// horizontal tracking wheel
lemlib::TrackingWheel horizontalTrackingWheel(&encoderHorizontal, lemlib::Omniwheel::OLD_275, -2.62277 /*Change me based off new position (3rd from back on wheel and 5th from wheel side on pivot point on drivebase)*/);
// vertical tracking wheel
lemlib::TrackingWheel verticalTrackingWheel(&encoderVertical, lemlib::Omniwheel::OLD_275, 1.69606);

lemlib::Drivetrain drivetrain(&drivetrainLeft, // left motor group
                              &drivetrainRight, // right motor group
                              14.91850, // 14.91850 inch track width
                              lemlib::Omniwheel::OLD_325, // using old 3.25" omnis
                              333.3333, // drivetrain rpm is 333.3333
                              2 // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(&verticalTrackingWheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontalTrackingWheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateralController(0, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              1, // anti windup
                                              1, // small error range, in inches
                                              1, // small error range timeout, in milliseconds
                                              1, // large error range, in inches
                                              1, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angularController(0.3777, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              1, // anti windup
                                              1, // small error range, in degrees
                                              1, // small error range timeout, in milliseconds
                                              1, // large error range, in degrees
                                              1, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateralController, // lateral PID settings
                        angularController, // angular PID settings
                        sensors
);




 
void on_center_button() {}


void initialize() {
	pros::lcd::initialize(); // initialize brain screen
    // while (true) { // infinite loop
    //     // print measurements from the adi encoder
    //     // pros::lcd::print(0, "ADI Encoder Horizontal: %i", encoderHorizontal.get_value());
    //     // pros::lcd::print(1, "ADI Encoder Vertical: %i", encoderVertical.get_value());
    //     // pros::lcd::print(2, "Controller Left Y: %i", primaryController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    //     // pros::lcd::print(3, "Controller Left X: %i", primaryController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    //     // pros::delay(10); // delay to save resources. DO NOT REMOVE
    // }
}

 
// void disabled() {}

 
// void competition_initialize() {}

 
void autonomous() {
    chassis.turnToHeading(90, 1000);
}
 
void opcontrol() {
    while (true) {
        // get left y and right x positions
        int leftY = primaryController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = primaryController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        // move the robot
        chassis.arcade(leftX, leftY, false, 0.75);
        // test.move(127);
        pros::delay(25);
    }}