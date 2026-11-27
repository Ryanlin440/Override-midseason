#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include <algorithm>
#include <cmath>




void openClaw(int lowerMs = 250) {
    uint32_t start = pros::millis();
    while (pros::millis() - start < (uint32_t)lowerMs) {
        cascade.move(getCascadeInches() > 0.011 ? cascadeDownSpeed() : 0); // go down, stop at the bottom
        pros::delay(10);
    }
    claw.set_value(true); // open
    clawBool = true;
    moveCascadeTo(0); // keep going down to 0
};

void closeClawSearching(int timeoutMs = 1000, double cascadeLift = GRAB_LIFT_IN, int liftDelayMs = 0) {
     uint32_t start = pros::millis();
     pros::Task armTask([=]() {
        // same prediction + grab as L1Button() in main.cpp
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
                // grab: close (false = closed, same as main.cpp), wait, lift, done
                claw.set_value(false);
                clawBool = false;
                pros::delay(CLAW_CLOSE_MS + liftDelayMs); // always wait for the claw to close, plus any extra
                moveCascadeTo(cascadeLift);
                break;
            }
            pros::delay(10);
            }
        chassis.arcade(0, 0); // stop driving if it timed out without grabbing
        });

}

void closeClawReg(){
    claw.set_value(false);
    clawBool = false;
}

void toggleAuto(int timeoutMs = 1000) {
    uint32_t start = pros::millis();
    while (!toggleSeesOurs() && pros::millis() - start < (uint32_t)timeoutMs) {
        if (chassis.isInMotion()) start = pros::millis(); // still driving in: the timeout starts once we arrive
        toggleMotor.move_velocity(toggleSearchRpm());
        pros::delay(10);
    }
    toggleMotor.move_velocity(0); // brake mode HOLD keeps it there
}

// ---------------------------------------------------------------------------
// Autonomous routines
// ---------------------------------------------------------------------------

void movementNearSide(){
    //Setting up inital Pos
    chassis.setPose(-63, 0, 90);
    distanceReset(90, false, true);

    //Toggling

    //ask ryan

    // Scoring first pin
    chassis.turnToPoint(-56, 0, 500);
    chassis.moveToPoint(-56, 0, 500, {.minSpeed = 127});

    chassis.moveToPoint(-47, -16, 500);

    // Getting out of way
    chassis.moveToPoint(-52, -8, 500, {.forwards = false, .minSpeed = 127});

    // Getting first pin/cup stack & scoring
    chassis.turnToPoint(-28, -21, 500);
    chassis.moveToPoint(-28, -21, 500);

    chassis.turnToPoint(-41, -24, 500);
    chassis.moveToPoint(-41, -24, 500);

    // Getting second pin/cup stack & scoring
    chassis.moveToPoint(-31, -31, 500, {.forwards = false});
    chassis.turnToPoint(-43, -43, 500);
    chassis.moveToPoint(-43, -43, 500);

    chassis.turnToPoint(-47, -30, 500);
    chassis.moveToPoint(-47, -30, 500);
    

}
//near side
void NearSideAutonOneStack() {
    // Getting third pin/cup stack from wall & scoring
    chassis.moveToPoint(-47, -38, 500, {.forwards = false});
    chassis.turnToPoint(-58, -29, 500);
    chassis.moveToPoint(-58, -29, 500);

    chassis.moveToPoint(-68, -23, 500);

    chassis.turnToPoint(-54, -24, 500);
    chassis.moveToPoint(-54, -24, 500);

}
//far side
void FarSideAutonAWP() {
    //Setting up inital Pos
    chassis.setPose(0, -63, 0);
    distanceReset(0, true, false);

    //Toggling

    //ask ryan

    // Scoring first pin
    chassis.turnToPoint(0, -56, 500);
    chassis.moveToPoint(0, -56, 500, {.minSpeed = 127});

    chassis.moveToPoint(-16, -47, 500);

    // Getting out of way
    chassis.moveToPoint(-8, -47, 500, {.forwards = false, .minSpeed = 127});

    // Getting in position for wall stack
    chassis.turnToPoint(-23, -60, 500);
    chassis.moveToPoint(-23, -60, 500);

    // Getting wall stack
    chassis.turnToPoint(-24, -65, 500);
    chassis.moveToPoint(-24, -65, 500);

    // Clearing
    chassis.turnToPoint(0, -47, 500, {.forwards = false});
    chassis.moveToPoint(0, -47, 500, {.forwards = false});

    // Going to goal & scoring
    chassis.turnToPoint(16, -47, 500);
    chassis.moveToPoint(16, -47, 500);

    // The reverse

    // Getting out of way
    chassis.moveToPoint(8, -47, 500, {.forwards = false, .minSpeed = 127});

    // Getting in position for wall stack
    chassis.turnToPoint(23, -60, 500);
    chassis.moveToPoint(23, -60, 500);

    // Getting wall stack
    chassis.turnToPoint(24, -65, 500);
    chassis.moveToPoint(24, -65, 500);

    // Clearing
    chassis.turnToPoint(0, -47, 500, {.forwards = false});
    chassis.moveToPoint(0, -47, 500, {.forwards = false});

    // Going to goal & scoring
    chassis.turnToPoint(-16, -47, 500);
    chassis.moveToPoint(-16, -47, 500);

    // Clearing for middle stack
    chassis.moveToPoint(-9, -47, 500, {.forwards = false});

    // Grabbing middle stack
    chassis.turnToPoint(-21, -27, 500);
    chassis.moveToPoint(-21, -27, 500);

    // Going to last goal
    chassis.turnToPoint(-47, 16, 500);
    chassis.moveToPoint(-47, 16, 500);

    // Toggling
    chassis.turnToPoint(-56, 8, 500, {.forwards = false});
    chassis.moveToPoint(-56, 8, 500, {.forwards = false, .minSpeed = 127});

    chassis.moveToPoint(-63, 0, 500, {.forwards = false, .minSpeed = 127});

    // Moving away from toggle
    chassis.moveToPoint(-60, 0, 500, {.minSpeed = 127});
    pros::delay(10);
}

// -----------Helpers for Skills ---------------
void resettingForSkillsFirstGoal(){
    // Resetting
    chassis.moveToPoint(-47, -59, 1000, {.forwards = false});
    chassis.turnToPoint(-65, -59, 500);
    chassis.moveToPoint(-65, -59, 500);

    // Grabbing from matchloader
    chassis.turnToPoint(-65, -59, 500);
    chassis.moveToPoint(-65, -59, 500);

    // Resetting #2
    chassis.moveToPoint(-47, -59, 1000, {.forwards = false});
}

void skillsFirstGoal(int cascadeHeight){
    resettingForSkillsFirstGoal();

    // Scoring
    chassis.turnToPoint(-47, -30, 500);
    chassis.moveToPoint(-47, -30, 500);
}

void spinMoveToGoal(){
    // Spin moveeeee
    chassis.turnToPoint(-38, -47, 500, {.forwards = false, .minSpeed = 127});
    chassis.moveToPoint(-38, -47, 500, {.forwards = false});

    // Spin moveeeee part 2
    chassis.turnToPoint(-30, -47, 500);
    chassis.moveToPoint(-30, -47, 500);
}

void spinMoveReverse(){
    // Get into position
    chassis.moveToPoint(-38, -47, 500, {.forwards = false});

    // Spin moveeeee
    chassis.turnToPoint(-47, -59, 500, {.forwards = false, .minSpeed = 127});
    chassis.moveToPoint(-47, -59, 500, {.forwards = false});
    // Spin moveeeee part 2
    chassis.turnToPoint(-38, -47, 500);
    chassis.moveToPoint(-38, -47, 500);

    // Move to Matchloader
    chassis.turnToPoint(-65, -59, 500);
    chassis.moveToPoint(-65, -59, 500);

}

void scoreMiddle(int cascadeHeight){
    // Score in middle
    chassis.turnToPoint(0, 0, 500);
    chassis.moveToPoint(0, 0, 500);
}
// -----------Helpers for Skills ---------------

void SkillsAuton() {
    movementNearSide();
    // Loop #1 on near side Alliance goal, stack #1
    skillsFirstGoal(10);

    // This grabs from matchloader only the pin, ends at the middle point, stack #2
    resettingForSkillsFirstGoal();

    // Loop #1 on far side Alliance goal, stack #3
        //Spin move, ends at the goal
        spinMoveToGoal();

        //Spin move reverse, ends at the matchloader
        spinMoveReverse();

        // Goes to reset point.
        chassis.moveToPoint(-47, -59, 500, {.forwards = false});

    // Loop #2 on far side Alliance goal, stack #4
        //Spin move, ends at the goal
        spinMoveToGoal();

        //Spin move reverse, ends at the matchloader
        spinMoveReverse();

        // Goes to reset point.
        chassis.moveToPoint(-47, -59, 500, {.forwards = false});

    // Loop #3 on far side Alliance goal, stack #5
        //Spin move, ends at the goal
        spinMoveToGoal();

        //Spin move reverse, ends at the matchloader
        spinMoveReverse();

        // Goes to reset point.
        chassis.moveToPoint(-47, -59, 500, {.forwards = false});

    // Loop #4 on far side Alliance goal, stack #6 & last pin.
        //Spin move, ends at the goal
        spinMoveToGoal();

        //Spin move reverse, ends at the matchloader
        spinMoveReverse();

        // Goes to reset point.
        chassis.moveToPoint(-47, -59, 500, {.forwards = false});

    // Go toggle
    chassis.turnToPoint(-10, -59, 500, {.forwards = false});
    chassis.moveToPoint(-10, -59, 1000, {.forwards = false});

    // Touch the wall
    chassis.turnToPoint(0, -65, 500, {.forwards = false});
    chassis.moveToPoint(0, -65, 1000, {.forwards = false});

    // Go middle
    chassis.turnToPoint(10, -10, 500);
    chassis.moveToPoint(10, -10, 1000, {.minSpeed = 127});

    scoreMiddle(5);

    // Grab first stack
    chassis.turnToPoint(22, 22, 500);
    chassis.moveToPoint(22, 22, 500);

    scoreMiddle(7);

    // Grab second stack
    chassis.turnToPoint(46, 46, 500);
    chassis.moveToPoint(46, 46, 500);

    scoreMiddle(9);

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
    closeClawSearching(10000);

}
