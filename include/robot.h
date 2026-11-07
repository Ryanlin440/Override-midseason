/**
 * \file robot.h
 *
 * Shared handles for every device and subsystem on the robot, plus the entry
 * point into the autonomous routes.
 *
 * The objects themselves are DEFINED once in src/main.cpp. This header only
 * declares them so other translation units (src/autons.cpp, etc.) can use them.
 *
 * Routes live in src/autons.cpp and are selected inside runAuton(), so adding
 * a new one never requires touching this header.
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <mutex>

#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern lemlib::Drivetrain drivetrain;
extern lemlib::Chassis chassis;

extern pros::Distance frontSensor;
extern pros::Distance leftSensor;
extern pros::Distance rightSensor;
extern pros::Distance backBottomSensor;
extern pros::Distance backTopSensor;
extern pros::Distance claw_sensor;


// The V5 IMU under-reads turns slightly, and by a different amount each way.
// This scales its rotation so LemLib's odom (which reads the IMU through a
// pros::Imu*) gets true degrees. Each change in rotation is scaled by its own
// direction and added to a running total. Measured with spinTest() at full
// speed: 10 turns left read ~3588, right ~3581 instead of 3600.
class ScaledImu : public pros::Imu {
  public:
    ScaledImu(std::uint8_t port, double leftScale, double rightScale)
        : pros::Imu(port), leftScale(leftScale), rightScale(rightScale) {}
    double get_rotation() const override {
        double raw = pros::Imu::get_rotation();
        if (!std::isfinite(raw)) return raw; // still calibrating / error
        std::lock_guard<pros::Mutex> lock(mutex);
        double delta = raw - lastRaw;
        // positive = clockwise = right turn
        scaled += delta * (delta > 0 ? rightScale : leftScale);
        lastRaw = raw;
        return scaled;
    }
    double get_heading() const override {
        double heading = std::fmod(get_rotation(), 360.0);
        return heading < 0 ? heading + 360.0 : heading;
    }
    // these jump the raw rotation, so the running total has to follow
    std::int32_t reset(bool blocking = false) const override {
        std::int32_t result = pros::Imu::reset(blocking);
        zero();
        return result;
    }
    std::int32_t tare_rotation() const override {
        std::int32_t result = pros::Imu::tare_rotation();
        zero();
        return result;
    }
  private:
    void zero() const {
        std::lock_guard<pros::Mutex> lock(mutex);
        lastRaw = 0;
        scaled = 0;
    }
    double leftScale;
    double rightScale;
    mutable pros::Mutex mutex;
    mutable double lastRaw = 0;
    mutable double scaled = 0;
};
extern ScaledImu imu;
extern pros::Rotation vertical_rotation_sensor;
extern pros::Rotation horizontal_rotation_sensor;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::OdomSensors sensors;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ExpoDriveCurve steer_curve;

extern pros::MotorGroup cascade;
extern pros::Rotation cascade_sensor;
extern lemlib::PID cascade_pid;
//void moveCascadeTo(double target);
void moveCascadeTo(double target, int timeoutMs = 2000);


constexpr float HS_AXLE_DIAMETER = 0.25f; 
// cascade stages multiply travel; calibrated so brain-screen inches match a ruler (8.5 in real move)
constexpr float CASCADE_TRAVEL_SCALE = 2.355f;
constexpr float INCHES_PER_DEGREE = (HS_AXLE_DIAMETER * 3.14159f) / 360.0f * CASCADE_TRAVEL_SCALE;

constexpr float cascadeRestHeight = 0;
float getCascadeInches();

extern pros::adi::DigitalOut clawShut;
extern bool clawBool;
extern pros::adi::DigitalOut leftToggle;
extern pros::adi::DigitalOut rightToggle;


// ---------------------------------------------------------------- controller
extern pros::Controller controller;

// ------------------------------------- autonomous (autons.cpp, auton_selector.cpp)
/// Runs the route picked with autonSelector(). Routes are in src/autons.cpp; the
/// selector and route list (autons[]) are in src/auton_selector.cpp.
void runAuton();
/// True while runAuton() is running. The driver button task in initialize() waits on it.
extern bool autonRunning;
/// Controller auton picker: Left/Right to choose, X to lock in. initialize() waits on it,
/// except on a competition switch/field, where it returns right away.
void autonSelector();
extern bool pickingAuton;
/// Brain left/right buttons step through the autons. Call after pros::lcd::initialize().
void registerBrainAutonSelector();
/// Controller Left/Right picker for competition_initialize(); runs until PROS ends it.
void competitionAutonSelector();
/// For autonomous(): sets the selected route's start pose (unless locked in with X), then runs it.
void runSelectedAuton();

// Tuning routines, callable from opcontrol() for testing.
void darwin_test();
void frictiontest();
void torquetest();
void tuningPID();
void spinTest();
void testing();
/// Thread-safe brain screen print for lines 5-7 (the screen task draws it).
/// Use this instead of pros::lcd::print anywhere outside the screen task.
void screenPrint(int line, const char* fmt, ...);

// Read by the screen task in main.cpp to show the spin test readout.
extern bool spinTestActive;
extern double spinTestRotation;
extern double spinTestPeakRate;


void L1Button();
void L2Button();
void R1Button();
void RightArrowButton();
void toggleMech();
void toggleMechUp();
void toggleMechDown();


void rotationChassisPID();
void linearChassisPID();

void closeClaw(int timeoutMs = 3000);
void openClaw();


void EastWestAuton();
void SkillsAuton();
void NorthSouthAuton();

void clampClaw();