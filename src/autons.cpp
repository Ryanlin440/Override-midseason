#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include <algorithm>



//for scoring only
void openClaw() {
    clawShut.set_value(false);
    clawBool=false;
    // pros::delay(500);
};




void closeClaw(int timeoutMs) {
     uint32_t start = pros::millis();
     pros::Task armTask([=]() {
        while (pros::millis() - start < (uint32_t)timeoutMs) {
            chassis.arcade(40,0);
            if (clawBool == false && claw_sensor.get_distance() < 50) {  
                chassis.cancelAllMotions();
                // 2. Clamp the object
                clawBool = true;
                clawShut.set_value(true);
                break;
            }
            pros::delay(30);
            }
        });

}





bool turningAgainstGoal = false;







// --------------------------------------- Autonomous routes

void movementRightSide(){
    moveCascadeTo(4,200);
    //Go score the preloaded pin
    chassis.turnToPoint(47, 18, 500, {});//uncommented to make faster

    chassis.moveToPoint(47, 17.8, 1000);
    pros::delay(500);
    moveCascadeTo(cascadeRestHeight,1000);
    openClaw();
    pros::delay(250);

    // Get out of the way and face towards the first cup/pin
    chassis.turnToPoint(47, 10, 200, {.forwards = false});
    chassis.moveToPoint(47,10, 400, {.forwards = false});

    chassis.turnToPoint(29.5, 19, 700);
    moveCascadeTo(cascadeRestHeight,1000);

    // Get the first cup/pin and get it
    chassis.moveToPoint(29.5, 19, 2000);

    
    pros::delay(250);
    closeClaw();//?either use sensors or just time it
    
    // Move to goal, move arm to back, and score
    chassis.turnToPoint(35, 30, 500, {.forwards = false, .maxSpeed= 60});
    chassis.moveToPoint(35, 30, 500, {.forwards = false});

    
    chassis.moveToPoint(44, 25, 800, {.forwards = false}, false);
    openClaw();//?
    // pros::delay(850);
    

    // second cup/pin and get it
    chassis.turnToPoint(47, 38, 2000);
    chassis.moveToPoint(47, 38, 2000);
    moveCascadeTo(cascadeRestHeight,1000);
    // pros::delay(1000);
    closeClaw();//?

    // // chassis.turnToHeading(315, 1000, {.maxSpeed = 30});
    chassis.turnToPoint(47, 30, 1000, {.forwards=false});
    chassis.moveToPoint(47, 30, 2000, {.forwards=false});
    pros::delay(500);
    openClaw();
}
//near side
void EastWestAuton() {
    // Toggle flip
    //move this back when there is wall - testing without wall
    // chassis.setPose(61.5,0,90);
    // TIMING: ms since auton start, on brain line 5 and the terminal. Remove once the start pause is found.
    // distanceReset(90,true,false);
    toggleMechDown();
    chassis.moveToPoint(58, 0, 400, {.forwards = false});
    chassis.moveToPoint(72, 0, 300);
    chassis.moveToPoint(58, 0, 400, {.forwards = false});
    chassis.moveToPoint(72, 0, 300);

    chassis.moveToPoint(47, 0, 1000, {.forwards = false});
    movementRightSide();

}
//far side
void NorthSouthAuton() {
    // Toggle flip
    //move this back when there is wall - testing without wall

    // distanceReset(0,false,true);
    // pros::delay(10000);
    toggleMechDown();
    chassis.moveToPoint(0, 58, 400, {.forwards = false});
    chassis.moveToPoint(0, 72, 300);
    chassis.moveToPoint(0, 58, 400, {.forwards = false});
    chassis.moveToPoint(0,72, 300);

    //getting ready to score preload
    chassis.moveToPoint(0, 47, 1000, {.forwards = false});
    // moveArmToHold(downArmDegPinAndCup, 1000, 60.0f);
    moveCascadeTo(3,200);
    //Go score the preloaded pin
    chassis.turnToPoint(18, 47, 500, {});//uncommented to make faster
    chassis.moveToPoint(18, 47, 1000);

    //score
    pros::delay(500);
    // moveArmToHold(0);
    moveCascadeTo(cascadeRestHeight,1000);
    openClaw();
    chassis.turnToPoint(10, 47, 200, {.forwards = false});
    pros::delay(150);

    // Get out of the way and face towards the first cup/pin
    chassis.moveToPoint(10,47, 400, {.forwards = false});
    // moveArmToHold(downArmDegPinAndCup);
    moveCascadeTo(cascadeRestHeight,1000);

    //move so that we can swing to position
    chassis.turnToPoint(13, 58.5, 1000);
    // moveArmToHold(downArmDegPinAndCup);
    moveCascadeTo(cascadeRestHeight,1000);
    chassis.moveToPoint(13, 58.5, 2000);           //can probably motion change this part
    chassis.turnToPoint(36, 58.5, 2000);
    chassis.moveToPoint(36, 58.5, 2000, {.earlyExitRange=1});
    chassis.swingToPoint(23, 60, lemlib::DriveSide::RIGHT, 700, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    //grab the middle stack
    chassis.turnToPoint(23, 59, 1000);
    chassis.moveToPoint(23, 59.1, 1000, {}, false);
    // pros::delay(10000);
    // chassis.arcade(20, 0);//make it keep going forward?
    // searchForStack(1000, cascadeRestHeight,150,80);
    closeClaw();
    chassis.arcade(30, 0);//make it keep going forward?
    pros::delay(250);
    chassis.moveToPoint(24, 50, 1000, {.forwards = false});
    //score the stack
    // pros::delay(250);
    // moveArmToBack();
    // openClawAndLiftThenPutBackFront(true);
    openClaw();
    pros::delay(2500);
    
    //go to the ohter side
    chassis.swingToPoint(-32, 56, lemlib::DriveSide::LEFT, 1000);

    chassis.moveToPoint(10, 56, 1000, {.minSpeed=70, .earlyExitRange=3});
    chassis.moveToPoint(-35, 56, 2000);

    //swing to correct heading
    chassis.swingToPoint(-24, 60, lemlib::DriveSide::LEFT, 700, {.direction =lemlib::AngularDirection::CW_CLOCKWISE});
    chassis.turnToPoint(-24, 58, 500);
    chassis.moveToPoint(-24, 58, 1000);

    // pros::delay(10000);
    // searchForStack(1000);
    closeClaw();
    chassis.arcade(30, 0);//make it keep going forward?
    pros::delay(250);
    //go into goal and score 
    chassis.moveToPoint(-24, 50, 1000, {.forwards = false});
    pros::delay(250);
    // moveArmToBack();
    // openClawAndLiftThenPutBackFront();
    openClaw();

        // chassis.moveToPoint(0, 38, 2000, {.minSpeed=40, .earlyExitRange = 3});




    // // Move back to goal and score
    // // chassis.moveToPoint(56, 45, 1000, {.forwards = false});
    // chassis.moveToPoint(55, 31, 1000, {.forwards = false});
    // chassis.moveToPoint(51, 25, 1000, {.forwards = false});
    // turnAgainstGoal(lemlib::AngularDirection::CW_CLOCKWISE, 90, 3500, true);
    // moveArmToBack();
    // openClawAndLiftThenPutBackFront();

    // // Swing to the third cup/pin and get it
    // chassis.moveToPoint(65, 29, 2000);
    // searchForStack(2000);

    // // Rotate to face neutral goal and go to it backwards, move arm to back and then score it.
    // chassis.turnToHeading(20, 1000);
    // chassis.moveToPoint(53, -20, 2000, {.forwards = false}, false);
    // turnAgainstGoal(lemlib::AngularDirection::CW_CLOCKWISE, 90, 3500, true);
    // moveArmToBack();
    // openClawAndLiftThenPutBackFront();

    // // Swing to the fourth cup/pin and get it
    // chassis.moveToPoint(68, -30, 2000);
    // closeClaw();

    // // Move back to goal and score
    // chassis.moveToPoint(54, -30, 2000, {.forwards = false});
    // moveArmToBack();
    // openClawAndLiftThenPutBackFront();

}


void SkillsAuton() {
    // Toggle flip
    //move this back when there is wall - testing without wall
    // chassis.setPose(65,0,90);
    // distanceReset(90,true,false);
    toggleMechDown();
    chassis.moveToPoint(58, 0, 400, {.forwards = false});
    // pros::delay(1000000);
    chassis.moveToPoint(72, 0, 300);
    chassis.moveToPoint(58, 0, 400, {.forwards = false});
    chassis.moveToPoint(72, 0, 300);

    chassis.moveToPoint(47, 0, 1000, {.forwards = false});
    // moveArmToHold(downArmDegPinAndCup, 1000, 60.0f);
    moveCascadeTo(6,1000);

    //Go score the preloaded pin
    chassis.turnToPoint(47, -16, 500, {});//uncommented to make faster

    chassis.moveToPoint(47, -16, 1000);
    pros::delay(500);//             do we need this one?
    moveCascadeTo(2.5,1000);
    openClaw();
    pros::delay(350);

    // Get out of the way and go towards match loader
    chassis.turnToPoint(47, -10, 500, {.forwards = false});
    chassis.moveToPoint(47,-10, 500, {.forwards = false});
    chassis.turnToPoint(58, -17, 500);
    chassis.moveToPoint(58,-17, 500);
    chassis.turnToPoint(60, -30, 500);
    chassis.moveToPoint(60,-30, 500);
    // moveArmToHold(downArmDegPinAndCup, 1000);
    moveCascadeTo(cascadeRestHeight,1000);

    // CLEARING
    chassis.turnToPoint(35, -59, 1000, {.forwards = false});
    chassis.moveToPoint(35, -59, 2000, {.forwards = false});
    chassis.turnToHeading(90, 1000);
    // distanceReset(NAN, true, true, 5, &leftSensor); // left sensor skipped 
    // Getting to matchloader
    chassis.turnToPoint(58.5, -58, 2000);
    chassis.moveToPoint(58.5, -58, 2000);
    // chassis.turnToPoint(58.5, -58, 2000);
    // searchForStackMatchLoad(1000, cascadeRestHeight+.2, 150, 55, true);
    closeClaw();
    
    pros::delay(250);
    //moveArmToHold(downArmDegPinAndCup+5);
    //chassis.moveToPoint(57, -58, 250);
    moveCascadeTo(cascadeRestHeight,1000);
    // moveArmTo(downArmDegPinAndCup, 600, 60.0f); 
    
    for(int i = 1; i <= 4; i++){
        // Moving to goal
        chassis.turnToPoint(47, -58, 1000, {.forwards = false});
        chassis.moveToPoint(47,-58, 1000, {.forwards = false});
        pros::delay(200);
        // primeArm();
        moveCascadeTo((i+1)*2);  //need to change this probably

        chassis.turnToPoint(47, -28, 1000, {.forwards = false});
        chassis.moveToPoint(47,-28, 1000, {.forwards = false}, false);

        // Scoring
        // distanceReset(NAN, true, true, 5, &rightSensor); // right sensor skipped
        // moveArmToBack();
        // openClawAndLiftThenPutBackFront(true);
        openClaw();
        pros::delay(250);
        // moveArmToHold(downArmDegPinAndCup);
        moveCascadeTo(cascadeRestHeight,1000);
        if(i == 4) break;

        // Get back in front of matchloader
        chassis.turnToPoint(47, -58, 1000);
        chassis.moveToPoint(47, -58, 1000);

        // Go into matchloader and get the next stack
        chassis.turnToPoint(59, -58, 1000);
        chassis.moveToPoint(59, -58, 1000);
        // chassis.turnToPoint(59, -58, 1000);

        // chassis.arcade(40,0);
        
        // searchForStackMatchLoad(1000, cascadeRestHeight+.2);
        closeClaw();
        pros::delay(250);
        //moveArmToHold(downArmDegPinAndCup+5);
        //chassis.moveToPoint(57, -58, 250);
        moveCascadeTo(cascadeRestHeight,1000);
        
    }

    //go park
    chassis.turnToPoint(47, -38, 1000);
    chassis.moveToPoint(47,-38, 1000);
    chassis.turnToPoint(6, -5, 1000, {.forwards=false});
    chassis.moveToPoint(6,-5, 2000, {.forwards=false});


    // // Get out of way.
    // chassis.turnToPoint(47, -40, 1000);
    // chassis.moveToPoint(47, -40, 1000);

    // // move to clear second batch
    // chassis.turnToPoint(28, -24, 1000, {.forwards = false});
    // chassis.moveToPoint(28,-24, 1000, {.forwards = false});

    // // get stack from mid
    // chassis.turnToPoint(24, -8, 1000);
    // chassis.moveToPoint(24, -8, 1000);
    // searchForStack(1000, cascadeRestHeight);
    // pros::delay(250);

    // seperatePin();
    
    // chassis.moveToPoint(47, 0, 1000, {.forwards = false}, false);

    // movementRightSide();

    // for (int i = 1; i <= 2; i++){
    //     // traveling to matchload
    //     chassis.turnToPoint(47, 58.5, 1000);
    //     chassis.moveToPoint(47, 58.5, 1000);
    //     chassis.turnToPoint(61, 58.5, 1000);
    //     chassis.moveToPoint(61, 58.5, 1000);
    //     searchForStack(1000, 2 * i);
    //     pros::delay(250);

    //     // Going to goal
    //     chassis.turnToPoint(47, 58.5, 1000, {.forwards = false});
    //     chassis.moveToPoint(47, 58.5, 1000, {.forwards = false});
    //     moveCascadeTo((i+2)*2);     //prob need to change this, i+2 cause there should be two cups already right?
    //     chassis.turnToPoint(47, 31, 1000, {.forwards = false});
    //     chassis.moveToPoint(47, 31, 1000, {.forwards = false});
    //     moveArmToBack();
        
    //     openClawAndLiftThenPutBackFront(true);
    //     pros::delay(250);
    // }
    // // prepare for middle
    // chassis.turnToPoint(47, 47, 1000);
    // chassis.moveToPoint(47, 47, 1000);

    // // going to middle
    // chassis.turnToPoint(8, 8, 1000, {.forwards = false});
    // chassis.moveToPoint(8, 8, 1000, {.forwards = false});
    // pros::delay(1000);
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
