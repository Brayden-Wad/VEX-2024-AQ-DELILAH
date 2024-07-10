#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"

//Defining devices
pros::Controller primaryController(pros::E_CONTROLLER_MASTER);
pros::Controller secondaryController(pros::E_CONTROLLER_PARTNER);
pros::Motor frontLeftDrive(8, pros::v5::MotorGears::green);
pros::Motor middleLeftDrive(9, pros::v5::MotorGears::green);
pros::Motor rearLeftDrive(10, pros::v5::MotorGears::green);
pros::Motor frontRightDrive(1, pros::v5::MotorGears::green);
pros::Motor middleRightDrive(2, pros::v5::MotorGears::green);
pros::Motor rearRightDrive(3, pros::v5::MotorGears::green);
pros::Motor intake(4, pros::v5::MotorGears::blue);
pros::Motor fourBar(7, pros::v5::MotorGears::green);
pros::adi::DigitalOut clamp('A');
pros::Imu inertialSensor(5);

//Motor groups
pros::MotorGroup leftDriveGroup({1, 2, 3});
pros::MotorGroup rightDriveGroup({8, 9, 10});


void on_center_button() {}


void initialize() {
    fourBar.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    fourBar.set_zero_position(0);
    inertialSensor.tare();
    pros::c::controller_print(pros::E_CONTROLLER_MASTER, 1, 1, "Cameron's controller");
    pros::c::controller_print(pros::E_CONTROLLER_PARTNER, 1, 1, "Brayden's controller");
}

void disabled() {}


void competition_initialize() {}


void autonomous() {}


void opcontrol() {
    intake.set_brake_mode(pros::MotorBrake::hold);
    fourBar.set_brake_mode(pros::MotorBrake::hold);
    while(true) {            
        //Driving
        /*Position 4*/int LX = primaryController.get_analog(ANALOG_LEFT_X);
        /*Position 1*/int RX = (primaryController.get_analog(ANALOG_RIGHT_X))*-1;
        /*Position 3*/int LY = primaryController.get_analog(ANALOG_LEFT_Y);   
        // Get the robot's current heading in radians
            double heading = inertialSensor.get_heading() * M_PI / 180.0;

        // Calculate the field-oriented values
            double tempForward = LY * cos(heading) + LX * sin(heading);
            double tempStrafe = -LY * sin(heading) + LX * cos(heading);

        //Driving
            frontRightDrive.move(RX - LX - LY);
            middleRightDrive.move(RX - LY);
            rearRightDrive.move(RX + LX - LY);
            frontLeftDrive.move(RX - LX*-1 + LY);
            middleLeftDrive.move(RX + LY);
            rearLeftDrive.move(RX + LX*-1 + LY);
            // int frontLeftSpeed = tempForward + tempStrafe + RX; This is for field centric driving
            // int frontRightSpeed = tempForward - tempStrafe - RX;
            // int rearLeftSpeed = tempForward - tempStrafe + RX;
            // int rearRightSpeed = tempForward + tempStrafe - RX;
            // frontLeftDrive.move(frontLeftSpeed);
            // rearLeftDrive.move(rearLeftSpeed);
            // frontRightDrive.move(frontRightSpeed);
            // rearRightDrive.move(rearRightSpeed);

        //Spin intake
            if (primaryController.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                intake.move(127);
            }
            else if (primaryController.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                intake.move(-127);
            }
            else if (secondaryController.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                intake.move(127);
            }
            else if (secondaryController.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                intake.move(-127);
            }
            else {
                intake.brake();
            }

        //fourBar movement
            if (primaryController.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                fourBar.move_absolute(500, 250);
            }
            else if (primaryController.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                fourBar.move_absolute(300, 250);
            }
            if (secondaryController.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                fourBar.move_absolute(500, 250);
            }
            else if (secondaryController.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                fourBar.move_absolute(300, 250);
            }
            else {
                fourBar.move_absolute(0, 150);
            }

        //Clamp movement
            if (primaryController.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                clamp.set_value(true);
            }
            else if (secondaryController.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                clamp.set_value(true);
            }
            if (primaryController.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                clamp.set_value(false);
            }
            else if (secondaryController.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                clamp.set_value(false);
            }
    }
}