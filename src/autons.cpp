#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include <algorithm>
#include <cmath>


// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

// Stack Heights
// Alliance
int justPinAlliance = 2;
int stack1Alliance = 4;
int stack2Alliance = 4;
int stack3Alliance = 4;
int stack4Alliance = 4;
int stack5Alliance = 4;

// Neutral
int justPinNeutral = 2;
int stack1Neutral = 4;
int stack2Neutral = 4;
int stack3Neutral = 4;
int stack4Neutral = 4;
int stack5Neutral = 4;

// Middle
int justPinMiddle = 2;
int stack1Middle = 4;
int stack2Middle = 4;
int stack3Middle = 4;

int waitTimeClearingMatchloader = 200;

// ---------------------------------------------------------------------------
// Helper functions for robot mechanics
// ---------------------------------------------------------------------------

void moveCascadeTo(double targetInches, int timeoutMs) {
    cascade_pid.reset();
    uint32_t start = pros::millis();

    while (true) {
        float current_pos_inches = getCascadeInches();
        float error = targetInches - current_pos_inches;
        float output = cascade_pid.update(error);

        // Below 0: the PID output is tiny this close to 0, so boost it to make sure it comes back up.
        constexpr float BELOW_ZERO_BOOST = 40.0f; // min upward power while below 0 - raise if it still sags
        if (current_pos_inches < 0) output = std::max(output, BELOW_ZERO_BOOST);

        output = std::clamp(output, -127.0f, 127.0f);
        cascade.move(output);

        // stop when at target (and not below 0) or the timer runs out
        bool atTarget = std::abs(error) < .3 && current_pos_inches >= 0;
        if (atTarget || pros::millis() - start > (uint32_t)timeoutMs) {
            cascade.set_brake_mode(pros::MotorBrake::hold);
            cascade.move_velocity(0);
            break;
        }
        pros::delay(10);
    }
}

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

void closeClawSearching(double cascadeLift = GRAB_LIFT_IN, int liftDelayMs = 0, int timeoutMs = 500) {
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
    moveCascadeTo(GRAB_LIFT_IN);
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
// Skills Helper Functions | Repetitive motion
// ---------------------------------------------------------------------------

void movementNearSide(){
    //Setting up inital Pos
    chassis.setPose(-63, 0, 90);
    distanceReset(90, false, true);

    //Toggling
    toggleAuto(1000);
    moveCascadeTo(justPinAlliance, 500);

    // Scoring first pin
    chassis.turnToPoint(-56, 0, 500);
    chassis.moveToPoint(-56, 0, 500, {.minSpeed = 127});

    chassis.moveToPoint(-47, -16, 500, {}, false);
    openClaw(100);

    // Getting out of way
    chassis.moveToPoint(-52, -8, 500, {.forwards = false, .minSpeed = 127});

    // Getting first pin/cup stack
    chassis.turnToPoint(-28, -21, 500);
    chassis.moveToPoint(-28, -21, 500);
    closeClawSearching(stack1Alliance);

    // Going back to goal
    chassis.turnToPoint(-41, -24, 500);
    chassis.moveToPoint(-41, -24, 500);
    openClaw();

    // Getting second pin/cup stack & scoring
    chassis.moveToPoint(-31, -31, 500, {.forwards = false});
    chassis.turnToPoint(-43, -43, 500);
    chassis.moveToPoint(-43, -43, 500);
    closeClawSearching(stack2Alliance);

    chassis.turnToPoint(-47, -30, 500);
    chassis.moveToPoint(-47, -30, 500);
    openClaw();
}

void resettingForSkillsFirstGoal(int height){
    // Resetting
    chassis.moveToPoint(-47, -59, 1000, {.forwards = false});
    chassis.turnToPoint(-65, -59, 500);
    chassis.moveToPoint(-65, -59, 500);

    // Grabbing from matchloader
    chassis.turnToPoint(-65, -59, 500);
    chassis.moveToPoint(-65, -59, 500, {}, false);
    closeClawReg();

    // Resetting #2
    chassis.moveToPoint(-47, -59, 1000, {.forwards = false});
    pros::delay(200);
    moveCascadeTo(height, 1000);
}

void skillsFirstGoal(){
    resettingForSkillsFirstGoal(stack3Alliance);

    // Scoring
    chassis.turnToPoint(-47, -30, 500);
    chassis.moveToPoint(-47, -30, 500, {}, false);
    openClaw();
}

void spinMoveToGoal(){
    // Spin moveeeee
    chassis.turnToPoint(-38, -47, 500, {.forwards = false, .minSpeed = 127});
    chassis.moveToPoint(-38, -47, 500, {.forwards = false});

    // Spin moveeeee part 2
    chassis.turnToPoint(-30, -47, 500);
    chassis.moveToPoint(-30, -47, 500, {}, false);
    openClaw();
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
    chassis.moveToPoint(-65, -59, 500, {}, false);
    closeClawReg();

}

void scoreMiddle(){
    chassis.turnToPoint(0, 0, 500);
    chassis.moveToPoint(0, 0, 500, {}, false);
    openClaw();
}

// ---------------------------------------------------------------------------
// Autonomous routines
// ---------------------------------------------------------------------------

void NearSideAutonOneStack() {
    // Getting third pin/cup stack from wall & scoring
    chassis.moveToPoint(-47, -38, 500, {.forwards = false});
    chassis.turnToPoint(-58, -29, 500);
    chassis.moveToPoint(-58, -29, 500);

    chassis.turnToPoint(-68, -23, 500);
    chassis.moveToPoint(-68, -23, 500, {}, false);
    closeClawReg();
    moveCascadeTo(stack3Alliance, 1000);

    chassis.turnToPoint(-47, -38, 500, {.forwards = false});
    chassis.moveToPoint(-47, -38, 500, {.forwards = false});

    chassis.turnToPoint(-47, -37, 500);
    chassis.moveToPoint(-47, -37, 500, {}, false);
    openClaw(250);
}

void FarSideAutonAWP() {
    //Setting up inital Pos
    chassis.setPose(0, -63, 0);
    distanceReset(0, true, false);

    //Toggling
    toggleAuto();
    moveCascadeTo(justPinAlliance, 500);

    // Scoring first pin
    chassis.turnToPoint(0, -56, 500);
    chassis.moveToPoint(0, -56, 500, {.minSpeed = 127});

    chassis.moveToPoint(-16, -47, 500, {}, false);
    openClaw(100);

    // Getting out of way
    chassis.moveToPoint(-8, -47, 500, {.forwards = false, .minSpeed = 127});

    // Getting in position for wall stack
    chassis.turnToPoint(-23, -60, 500);
    chassis.moveToPoint(-23, -60, 500);

    // Getting wall stack
    chassis.turnToPoint(-24, -65, 500);
    chassis.moveToPoint(-24, -65, 500, {}, false);
    closeClawReg();

    // Clearing
    chassis.turnToPoint(0, -47, 500, {.forwards = false});
    chassis.moveToPoint(0, -47, 500, {.forwards = false});
    moveCascadeTo(stack1Neutral, 500);

    // Going to goal & scoring
    chassis.turnToPoint(16, -47, 500);
    chassis.moveToPoint(16, -47, 500, {}, false);
    openClaw(250);

    // ---------- The reverse-----------

    // Getting out of way
    chassis.moveToPoint(8, -47, 500, {.forwards = false, .minSpeed = 127});

    // Getting in position for wall stack
    chassis.turnToPoint(23, -60, 500);
    chassis.moveToPoint(23, -60, 500);

    // Getting wall stack
    chassis.turnToPoint(24, -65, 500);
    chassis.moveToPoint(24, -65, 500, {}, false);
    closeClawReg();

    // Clearing
    chassis.turnToPoint(0, -47, 500, {.forwards = false});
    chassis.moveToPoint(0, -47, 500, {.forwards = false});
    moveCascadeTo(stack1Alliance, 500);

    // Going to goal & scoring
    chassis.turnToPoint(-16, -47, 500);
    chassis.moveToPoint(-16, -47, 500, {}, false);
    openClaw(250);

    // Clearing for middle stack
    chassis.moveToPoint(-9, -47, 500, {.forwards = false});

    // Grabbing middle stack
    chassis.turnToPoint(-21, -27, 500);
    chassis.moveToPoint(-21, -27, 500);
    closeClawSearching(stack1Neutral);

    // Going to last goal
    chassis.turnToPoint(-47, 16, 500);
    chassis.moveToPoint(-47, 16, 500, {}, false);
    openClaw(250);

    // Toggling
    chassis.turnToPoint(-56, 8, 500, {.forwards = false});
    chassis.moveToPoint(-56, 8, 500, {.forwards = false, .minSpeed = 127});

    chassis.moveToPoint(-63, 0, 500, {.forwards = false, .minSpeed = 127});
    toggleAuto(1000);

    // Moving away from toggle
    chassis.moveToPoint(-60, 0, 500, {.minSpeed = 127});
    pros::delay(10);
}

void SkillsAuton() {
    movementNearSide();
    // Loop #1 on near side Alliance goal, stack #1
    skillsFirstGoal();

    // This grabs from matchloader only the pin, ends at the middle point, stack #2
    resettingForSkillsFirstGoal(justPinAlliance);

    // Loop #1 on far side Alliance goal, stack #3
        //Spin move, ends at the goal
        spinMoveToGoal();

        //Spin move reverse, ends at the matchloader
        spinMoveReverse();
        pros::delay(waitTimeClearingMatchloader);
        moveCascadeTo(stack1Alliance, 500);

        // Goes to reset point.
        chassis.moveToPoint(-47, -59, 500, {.forwards = false});

    // Loop #2 on far side Alliance goal, stack #4
        //Spin move, ends at the goal
        spinMoveToGoal();

        //Spin move reverse, ends at the matchloader
        spinMoveReverse();
        pros::delay(waitTimeClearingMatchloader);
        moveCascadeTo(stack2Alliance, 500);

        // Goes to reset point.
        chassis.moveToPoint(-47, -59, 500, {.forwards = false});

    // Loop #3 on far side Alliance goal, stack #5
        //Spin move, ends at the goal
        spinMoveToGoal();

        //Spin move reverse, ends at the matchloader
        spinMoveReverse();
        pros::delay(waitTimeClearingMatchloader);
        moveCascadeTo(stack3Alliance, 1000);

        // Goes to reset point.
        chassis.moveToPoint(-47, -59, 500, {.forwards = false});

    // Loop #4 on far side Alliance goal, stack #6.
        //Spin move, ends at the goal
        spinMoveToGoal();

        //Spin move reverse, ends at the matchloader
        spinMoveReverse();
        pros::delay(waitTimeClearingMatchloader);
        moveCascadeTo(stack4Alliance, 1000);

        // Goes to reset point.
        chassis.moveToPoint(-47, -59, 500, {.forwards = false});

    // Loop #5 on far side Alliance goal, last pin.
        //Spin move, ends at the goal
        spinMoveToGoal();

        //Spin move reverse, ends at the matchloader
        spinMoveReverse();
        pros::delay(waitTimeClearingMatchloader);
        moveCascadeTo(justPinMiddle, 500);

        // Goes to reset point.
        chassis.moveToPoint(-47, -59, 500, {.forwards = false});

    // Go toggle
    chassis.turnToPoint(-10, -59, 500, {.forwards = false});
    chassis.moveToPoint(-10, -59, 1000, {.forwards = false});

    // Touch the wall
    chassis.turnToPoint(0, -65, 500, {.forwards = false});
    chassis.moveToPoint(0, -65, 500, {.forwards = false}, false);
    toggleAuto();

    // Go middle
    chassis.turnToPoint(10, -10, 500);
    chassis.moveToPoint(10, -10, 1000, {.minSpeed = 127});

    scoreMiddle();

    // Grab first stack
    chassis.turnToPoint(22, 22, 500);
    chassis.moveToPoint(22, 22, 500);
    closeClawSearching(stack1Middle);

    scoreMiddle();

    // Grab second stack
    chassis.turnToPoint(46, 46, 500);
    chassis.moveToPoint(46, 46, 500);
    closeClawSearching(stack2Middle);

    scoreMiddle();

}

// ---------------------------------------------------------------------------
// Test / tuning 
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

// IMU scale test
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
