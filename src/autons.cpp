#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include <algorithm>
#include <cmath>



//for scoring only
void openClaw() {
    claw.set_value(true);
    clawBool=true;
    // pros::delay(500);
};


void closeClaw(int timeoutMs) {
     uint32_t start = pros::millis();
     pros::Task armTask([=]() {
        // same prediction + grab as L1Button() in main.cpp
        constexpr double GRAB_LIFT_IN = 3; // how far to lift after grabbing
        constexpr int    GRAB_DIST_MM = 50; // minimum dist
        constexpr double CLAW_CLOSE_MS = 40; // how long it takes to actually close
        constexpr int    PREDICT_MAX_MM = 304; // a distance close enough for readings ~ 12 inches
        constexpr double MIN_APPROACH = 0.05; // m/s, ignore noise below this

        // true if the object will be in the claw by the time the pneumatics finish closing
        auto shouldClose = [&]() {
            int dist = clawSensor.get_distance();
            if (dist <= 0 || dist >= 9999) return false;   // no object / error
            if (dist <= GRAB_DIST_MM) return true;
            if (dist > PREDICT_MAX_MM) return false;
            double approach =  clawSensor.get_object_velocity();
            if (approach == PROS_ERR_F || !std::isfinite(approach)) return false;
            if (approach < MIN_APPROACH) return false;
            return (dist - GRAB_DIST_MM) / approach < CLAW_CLOSE_MS; // ms until it reaches grab point
        };

        while (pros::millis() - start < (uint32_t)timeoutMs) {
            chassis.arcade(40,0);
            if (shouldClose()) {
                chassis.cancelAllMotions();
                chassis.arcade(0, 0);
                // grab: close (false = closed, same as main.cpp), wait for it, lift, done
                claw.set_value(false);
                clawBool = false;
                pros::delay(CLAW_CLOSE_MS);
                moveCascadeTo(GRAB_LIFT_IN);
                break;
            }
            pros::delay(10);
            }
        chassis.arcade(0, 0); // stop driving if it timed out without grabbing
        });

}

bool turningAgainstGoal = false;


// ---------------------------------------------------------------------------
// Autonomous routines
// ---------------------------------------------------------------------------

void movementRightSide(){
    
}
//near side
void EastWestAuton() {

}
//far side
void NorthSouthAuton() {

}


void SkillsAuton() {
    
}
// ---------------------------------------------------------------------------
// Test / tuning routines
// ---------------------------------------------------------------------------

void darwin_test() {
    int counter = 0;
    chassis.setPose(0, 0, 0);
    int secondVar = 0;
    while (frontSensor.get() > 500) {
        chassis.arcade(127, 0);
        counter++;
        screenPrint(5, "%d sec: %f", counter, chassis.getPose().y);
        pros::delay(100);
    }
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.arcade(0, 0);
}

void frictiontest() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.arcade(127, 0);
    pros::delay(500);
    chassis.arcade(0, 0);
    screenPrint(6, "Final Velocity: %.4f", vertical_rotation_sensor.get_velocity());
    int count = 0;
    while (vertical_rotation_sensor.get_velocity() > 1) {
        count++;
        pros::delay(50);
    }
    screenPrint(7, "Time till stop: %.4f seconds",
                     vertical_rotation_sensor.get_velocity() * 20); // 20 = 1000/50 or 1 second / milli
}

void torquetest() {
    int torqueCount = 47;
    while (true) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            torqueCount += 10;
            if (torqueCount > 127) torqueCount = 127;
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            torqueCount -= 10;
            if (torqueCount < 0) torqueCount = 0;
        }
        screenPrint(5, "Current voltage: %d", torqueCount);
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            left_motors.move(torqueCount);
        } else {
            left_motors.move(0);
        }
    }
}

void tuningPID(){
    chassis.turnToHeading(90, 2000);
    pros::delay(4000);
    chassis.turnToHeading(180, 2000);
    pros::delay(4000);
    chassis.turnToHeading(270, 2000);
    pros::delay(4000);
    chassis.turnToHeading(0, 2000);

    // chassis.moveToPoint(0,12, 2000);
    // pros::delay(4000);
    // chassis.moveToPoint(0,24, 2000);
    // pros::delay(4000);
    // chassis.moveToPoint(0,48, 2000);
    // pros::delay(4000);
    // chassis.moveToPoint(0,0, 2000, {.forwards=false});


}

// IMU scale test. Blocks opcontrol until B is pressed: only right-stick
// turning works while it runs. Line the robot up against a wall/tile edge,
// press A to zero, spin in place with the right stick (or by hand), and line
// it back up after some whole number of turns. The screen task in main.cpp
// shows the rotation and the scale factor (true degrees / IMU degrees).
// This never prints to the brain screen itself: LVGL isn't thread-safe, and
// printing from here and the screen task at once crashes the brain.
bool spinTestActive = false;
double spinTestRotation = 0;
double spinTestPeakRate = 0; // fastest turn rate seen since the last zero, deg/s

void spinTest() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // Use an offset instead of tare_rotation() so LemLib's odom isn't disturbed.
    double start = imu.get_rotation();
    spinTestRotation = 0;
    spinTestPeakRate = 0;
    spinTestActive = true;
    while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            start = imu.get_rotation();
            spinTestPeakRate = 0;
        }
        chassis.arcade(0, controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        spinTestRotation = imu.get_rotation() - start;
        double rate = std::fabs(imu.get_gyro_rate().z);
        if (std::isfinite(rate) && rate > spinTestPeakRate) spinTestPeakRate = rate;
        pros::delay(20);
    }
    chassis.arcade(0, 0);
    spinTestActive = false;
}

void testing(){
    closeClaw(10000);

}
