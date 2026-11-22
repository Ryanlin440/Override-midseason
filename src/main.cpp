#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/optical.h"
#include "robot.h"
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
void L1Button();
pros::MotorGroup right_motors({-11, 12, 13}, pros::MotorGearset::blue);
pros::MotorGroup left_motors({-20, -19, 18}, pros::MotorGearset::blue); 
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12.6, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// pros::Distance backSensor(20);
pros::Distance frontSensor(15);
pros::Distance leftSensor(7);
pros::Distance rightSensor(1);

// Which device goes with each sensor name in include/localization/robot_config.hpp (positions live
// there). Used by the UKF, distanceReset() and offsetTuner(). Add a line when you add a sensor.
pros::Distance* distanceSensorFor(const char* name) {
    if (std::strcmp(name, "front") == 0) return &frontSensor;
    if (std::strcmp(name, "right") == 0) return &rightSensor;
    if (std::strcmp(name, "left") == 0) return &leftSensor;
    return nullptr;
}

ScaledImu imu(6, 1, 1); // left, right scales from spinTest(); right averages half + full speed runs
// ScaledImu imu(5, 1.0000, 1.0000); // left, right scales from spinTest(); right averages half + full speed runs
pros::Rotation vertical_rotation_sensor(-10);
pros::Rotation horizontal_rotation_sensor(-9);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation_sensor, lemlib::Omniwheel::NEW_2, -2.55);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rotation_sensor, lemlib::Omniwheel::NEW_2, -4.5);

pros::MotorGroup cascade ({-16, 15}, pros::v5::MotorGears::blue);//2 11w
pros::Rotation cascade_sensor(-3);
lemlib::PID cascade_pid(48.7,4,389,0); // old (5,0,20) per degree, scaled to per inch
float getCascadeInches() {
    return (cascade_sensor.get_position() / 100.0f) * INCHES_PER_DEGREE;
}
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
pros::Motor toggleMotor(-8,pros::v5::MotorGears::green);
pros::Optical toggleSensor(2);
bool clawBool = false;
pros::adi::DigitalOut claw(4, clawBool);
pros::Distance clawSensor(17);

pros::adi::DigitalOut toucherLeft(2, false);
pros::adi::DigitalOut toucherRight(1, false); ///////////////////

pros::adi::DigitalOut wedge(3, false);
bool wedgeBool = false;
bool toucherLeftBool = false;
bool toucherRightBool = false;



lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(10.16, // proportional gain (kP)
                                              .024672, // integral gain (kI)
                                              277.163, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// lemlib::ControllerSettings lateral_controller(6, // proportional gain (kP)
//                                               0.01, // integral gain (kI)
//                                               30, // derivative gain (kD)
//                                               5, // anti windup
//                                               1, // small error range, in inches
//                                               100, // small error range timeout, in milliseconds
//                                               3, // large error range, in inches
//                                               500, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );

// angular PID controller
lemlib::ControllerSettings angular_controller(1.742, // proportional gain (kP)
                                              .07687, // integral gain (kI)
                                              26.112, // derivative gain (kD)
                                              5, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                            3, // large error range, in degrees
                                              300, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.025 // expo curve gain
);
//both previously 1.019
// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.025 // expo curve gain
);
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
                        &throttle_curve, 
                        &steer_curve
);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
// The brain screen (LVGL) isn't thread-safe: calling pros::lcd from two tasks
// at once corrupts it and can crash the brain (DATA ABORT in lv_obj_style.c).
// So only the screen task in initialize() draws. Everything else calls
// screenPrint(), which saves the text for the screen task to draw on lines 3-7.
// Line map is in robot.h next to SCREEN_AUTON.
constexpr int FIRST_MESSAGE_LINE = SCREEN_AUTON;
constexpr int MESSAGE_LINES = 8 - FIRST_MESSAGE_LINE;
pros::Mutex screenMutex;
char screenMessages[MESSAGE_LINES][48] = {};

void screenPrint(int line, const char* fmt, ...) {
    if (line < FIRST_MESSAGE_LINE || line > 7) return;
    char buf[48];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    screenMutex.take();
    std::strcpy(screenMessages[line - FIRST_MESSAGE_LINE], buf);
    screenMutex.give();
}

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
int pressNum = 0;
// Cleared by turnTunerAuton() so the screen task stops overwriting its readout.
bool printingDistances = true;
bool armNeedsDown = true;
bool isScoring = false;
int opticalRefreshRate = 10; // ms sensor integration time; lower = fresher color readings
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    registerBrainAutonSelector(); // brain left/right buttons pick the auton
    chassis.calibrate(); // calibrate sensors
    cascade_sensor.reset_position();
    cascade.move_velocity(0);


    // So we aren't touching the toggle when match starts
    //light it up, set up refresh rate, somthing abt gestures
    toggleSensor.set_led_pwm(100);
    pros::delay(200);
    toggleSensor.set_integration_time(opticalRefreshRate);//prob make lower if it doens't stop fast enough
    toggleSensor.disable_gesture();
    toggleMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // pros::delay(100);

    startAutoToggle(); // always-on color sort; set autoToggleEnabled = false to drive toggleMotor yourself

    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            if (not printingDistances) {
                pros::delay(40);
                continue;
            }
            if (spinTestActive) {
                double rotation = spinTestRotation;
                double turns = std::round(std::fabs(rotation) / 360.0);
                pros::lcd::print(0, "SPIN TEST  A=zero B=exit");
                pros::lcd::print(1, "Rotation: %.2f", rotation);
                pros::lcd::print(2, "Turns: %d", (int)turns);
                if (turns > 0) {
                    pros::lcd::print(3, "Scale: %.5f", turns * 360.0 / std::fabs(rotation));
                } else {
                    pros::lcd::print(3, "Scale: spin 1+ turns");
                }
                pros::lcd::print(4, "Peak rate: %.0f deg/s", spinTestPeakRate);
                pros::delay(100);
                continue;
            }
            lemlib::Pose pose = chassis.getPose();
            double heading = std::fmod(pose.theta, 360.0);
            if (heading < 0) heading += 360.0;

            // ---- controller: line 0 = sort color, line 2 = pose.
            // The controller drops prints sent less than ~50 ms apart (from ANY task), so send
            // at most one print per 60 ms and alternate between the two lines.
            // Skip while someone else owns the controller: the selector (picking, or disabled on
            // a switch/field where competitionAutonSelector() runs) or a tuning tool.
            bool fieldDisabled = pros::competition::is_connected() && pros::competition::is_disabled();
            static uint32_t lastControllerPrint = 0;
            static bool poseTurn = false;
            if (!pickingAuton && !fieldDisabled && !tuningToolActive &&
                pros::millis() - lastControllerPrint >= 60) {
                if (poseTurn) {
                    controller.print(2, 0, "%5.1f %5.1f %3.0f ", pose.x, pose.y, heading);
                } else {
                    controller.print(0, 0, "Sort %-4s %-5s", isRed ? "RED" : "BLUE",
                                     autoToggleEnabled ? "auto" : "manual");
                }
                poseTurn = !poseTurn;
                lastControllerPrint = pros::millis();
            }

            // ---- brain lines 0-2 (screen task's own)
            pros::lcd::print(0, "X %7.2f  Y %7.2f  H %6.2f", pose.x, pose.y, heading);
            pros::lcd::print(1, "Front %.2f in   Cascade %.3f in",
                             (double)(frontSensor.get() / 25.4), (double)getCascadeInches());
            pros::lcd::print(2, "Sort %s (%s)  hue %3.0f  prox %3d  trv %4.0f", isRed ? "RED" : "BLUE",
                             autoToggleEnabled ? "auto" : "manual", toggleSensor.get_hue(),
                             (int)toggleSensor.get_proximity(), toggleLastTravel);

            // ---- brain lines 3-7 (screenPrint messages)
            char messages[MESSAGE_LINES][48];
            screenMutex.take();
            std::memcpy(messages, screenMessages, sizeof(messages));
            screenMutex.give();
            for (int i = 0; i < MESSAGE_LINES; i++) pros::lcd::print(FIRST_MESSAGE_LINE + i, "%s", messages[i]);
            pros::delay(40);
            //
        }
    });


    // if (USE_UKF) startUkf(); // UKF owns the pose; status on line 7
    // else startPoseFusion(); // blends distance sensors into odom; gains on line 7

    pros::Task cascadeTask([&]() {
        while (pickingAuton) pros::delay(20); // driver buttons do nothing until X
        while (true) {
            // auton drives the arm itself; R1Button's arm hold would fight it.
            // Also skip while disabled on a switch/field: the controller arrows are picking
            // the auton then, and Right would otherwise open the claw and drop the preload.
            bool fieldDisabled = pros::competition::is_connected() && pros::competition::is_disabled();
            if (autonRunning || fieldDisabled) {
                pros::delay(20);
                continue;
            }
         
            

            pros::delay(50);
            }
        });

    autonSelector(); 
    if (!pros::competition::is_connected()) {
        runAuton();
    }

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opontrol
 * starts.
 */
void competition_initialize() {
    competitionAutonSelector();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() { 

    poseFusionEnabled = true;
    ukfEnabled = true;
    // runs whatever the brain/controller selector shows, from its start pose in autons[]
    runSelectedAuton();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void rotationChassisPID(){
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 1000);
    pros::delay(4000);
    chassis.turnToHeading(180, 1000);
    pros::delay(4000);
    chassis.turnToHeading(360, 1000);
    pros::delay(4000);
}

void linearChassisPID(){
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 2000);
    pros::delay(5000);
    chassis.moveToPoint(0, 48, 2000);
    pros::delay(5000);
    chassis.moveToPoint(0, 0, 2000, {.forwards=false});
    pros::delay(5000);
}

bool L1Bool = false;
float cascadeDownSpeed(){
    // float a = fmax(0, getCascadeInches());
    // return -120*pow(a,.3);
    if(getCascadeInches() < 0)return 0;
    return -127;
}
void opcontrol() {
    // the field kills autonomous() at 15 s, so it never gets to clear this itself
    autonRunning = false;
    poseFusionEnabled = false;
    // ukfEnabled = true; // UKF keeps running in driver control

    // tuneCascadePID(5,80); // blocks ~15 s and L1 does nothing until it's done - only enable to tune
    // tuneAngularPID(90,127);
    // tuneLateralPID(24,127);
    // pros::delay(10000000);
    /*
    // loop forever
    // autonomous();

    // rotationChassisPID();
    // linearChassisPID();
    
    // return;
    // while(true){
    //     arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //     arm_motor.move_velocity(0);
    // }
    */

    // spinTest();

    // tuningPID();
    // pros::delay(10000);
    // tuningPID();
    // pros::delay(1000000);
    
    // autonomous();
    L1Button();
    while (true) {

        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);
        // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
        //     moveCascadeTo(10);
        // }else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        //     moveCascadeTo(0);
        // }


        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            wedgeBool = !wedgeBool;
            wedge.set_value(wedgeBool);
        }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
            toucherRightBool = !toucherRightBool;
            toucherRight.set_value(toucherRightBool);
        }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            toucherLeftBool = !toucherLeftBool;
            toucherLeft.set_value(toucherLeftBool);
        }
        manualToggle();
        double cascadePos = getCascadeInches();
        
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            cascade.move(127); // Move Up
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            // Only allow reverse if it is above 0
            if (cascadePos > 0.011) {
                cascade.move(cascadeDownSpeed());
            } else {
                cascade.move(0); 
            }
        }
        else{
            if(!L1Bool)
                cascade.move(0); 
        }
        
        
        
        // delay to save resources
        pros::delay(25);
    }
}



void L1Button(){
    // toggleMotor.move(-50);
    //     pros::delay(2000);
    pros::Task clawTask([]() {
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

        auto grab = []() {
            L1Bool = true;                 
            claw.set_value(false); 
            clawBool = false;
            pros::delay(CLAW_CLOSE_MS);
            moveCascadeTo(GRAB_LIFT_IN);
            // pros::delay(1000);
            L1Bool = false;
        };

        bool wasPressed = false;
        bool grabbedThisPress = false; 
        bool lowering = false;     

        while (true) {
            bool pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
            bool justPressed  = pressed && !wasPressed;
            bool justReleased = !pressed && wasPressed;
            wasPressed = pressed;

            if (justPressed) {
                grabbedThisPress = false;
                lowering = !clawBool; // decide mode based on claw state at press time
            }

            if (lowering) {//open
                if (pressed) {//go down
                    L1Bool = true;//block the move(0)
                    cascade.move(getCascadeInches() > 0.011 ? cascadeDownSpeed() : 0);
                } else if (justReleased) {
                    claw.set_value(true); // open
                    clawBool = true;
                    moveCascadeTo(0);
                    L1Bool = false;
                    lowering = false;//finished lowering cascade
                }
            } else if (clawBool) { // open
                if (pressed && !grabbedThisPress && shouldClose()) {
                    grab();
                    grabbedThisPress = true; // ignore the eventual release
                } else if (justReleased && !grabbedThisPress) {
                    grab();
                }
            }

            pros::delay(10);
        }
    });
}
void manualToggle() {
    // Hold R1 to run the toggle by hand; the auto toggle task backs off while it's held.
    static bool manual = false;
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        manual = true;
        autoToggleEnabled = false;
        toggleMotor.move(100);
        // gaveUp=true;
    } else if (manual) { // just released: stop and hand control back to the task
        manual = false;
        toggleMotor.move(0);
        autoToggleEnabled = true;
    }
}


bool autoToggleEnabled = true;


// Color sort: spin at a fixed speed until our color is in front of the sensor, then stop and hold.
// "trv" on brain line 2 = degrees the toggle coasted after seeing our color (how far it overshot
// because of sensor lag + braking). If it keeps stopping past our color, lower kSearchSpeedFrac.
constexpr double kSearchSpeedFrac = 0.8; // fraction of max rpm while searching - lower = more accurate stop
constexpr uint32_t kMaxSpinMs = 5000;    // give up on an object after this long (jammed / no colors seen)
constexpr uint32_t kWrongMs = 150;       // after stopping, re-sort if it reads "not ours" this long (someone turned it)
double toggleLastTravel = 0;

static bool hueIsRed(double hue) { return hue > 350 || hue < 35; }
static bool hueIsBlue(double hue) { return hue > 100 && hue < 240; }

static int toggleMaxRpm() {
    switch (toggleMotor.get_gearing()) {
        case pros::v5::MotorGears::red: return 100;
        case pros::v5::MotorGears::blue: return 600;
        default: return 200;
    }
}

void startAutoToggle() {
    pros::Task toggleTask([]() {
        enum class State { Idle, Search, Done };
        State state = State::Idle;
        uint32_t stateStart = 0;
        double stopPos = 0;
        uint32_t stopTime = 0, notOursSince = 0;
        bool gaveUp = false;
        const int searchRpm = (int)(toggleMaxRpm() * kSearchSpeedFrac);

        while (pickingAuton) pros::delay(20);

        while (true) {
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                isRed = !isRed;
            } // toggles colors for testing

            if (!autoToggleEnabled) { // don't touch it - we r manually spinning
                state = State::Idle;
                gaveUp = false;
                pros::delay(10);
                continue;
            }

            bool objectPresent = toggleSensor.get_proximity() > 100; // check this value
            double hue = toggleSensor.get_hue();
            bool seesRed = hueIsRed(hue), seesBlue = hueIsBlue(hue);
            bool seesOurs = isRed ? seesRed : seesBlue;
            bool seesOpposite = isRed ? seesBlue : seesRed;
            uint32_t now = pros::millis();

            if (!objectPresent) { // object left: reset
                gaveUp = false;
                if (state != State::Idle) toggleMotor.move_velocity(0);
                state = State::Idle;
            }

            switch (state) {
                case State::Idle:
                    if (objectPresent && !gaveUp) {
                        if (seesOurs) {
                            toggleMotor.move_velocity(0); // already right, hold
                        } else {
                            state = State::Search;
                            stateStart = now;
                        }
                    }
                    break;

                case State::Search:
                    toggleMotor.move_velocity(searchRpm);
                    if (seesOurs) {
                        toggleMotor.move_velocity(0); // brake mode HOLD keeps it there
                        stopPos = toggleMotor.get_position();
                        stopTime = now;
                        notOursSince = 0;
                        state = State::Done;
                    } else if (now - stateStart > kMaxSpinMs) {
                        toggleMotor.move_velocity(0);
                        gaveUp = true; // don't retry until the object leaves
                        state = State::Idle;
                    }
                    break;

                case State::Done:
                    // Holding on ours. Keep watching: if it overshot, or a human turns it off our
                    // color later, sort again.
                    if (now - stopTime < 300) toggleLastTravel = toggleMotor.get_position() - stopPos; // coast only
                    if (seesOurs) {
                        notOursSince = 0;
                    } else if (notOursSince == 0) {
                        notOursSince = now;
                    }
                    // Opposite color = re-sort right away. No color / edge = wait kWrongMs so a
                    // noisy reading or a face edge doesn't make it twitch.
                    if (seesOpposite || (notOursSince != 0 && now - notOursSince > kWrongMs)) {
                        state = State::Search;
                        stateStart = now;
                    }
                    break;
            }

            pros::delay(10);
        }
    });
}
