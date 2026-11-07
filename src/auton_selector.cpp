#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include <algorithm>
#include <cstdint>

// ---------------------------------------------------------------------------
// Auton selector -- shown on controller line 2 and brain line 6. Left/Right on
// the controller or the brain's left/right buttons to choose; both stay in sync.
//   1st X: locks in the route and sets its start pose. Odometry keeps
//          tracking from there, so the robot can be pushed into place.
//   2nd X: starts the route (only without a competition switch/field; on a
//          switch/field, autonomous() starts it instead).
// To add a route, write the function in autons.cpp, declare it in robot.h, and
// add a line here with its start pose (keep Testing last).
// ---------------------------------------------------------------------------

struct AutonRoute {
    const char* name;
    void (*run)(); // nullptr = no auton, straight to driver control
    float x, y, theta; // start pose set on the first X press
};

AutonRoute autons[] = {
    {"Driver", nullptr, 0, 0, 0},
    {"EastWest Auton", EastWestAuton, 61.5, 0, 90},
    {"NorthSouth Auton", NorthSouthAuton, 0, 61.5, 0},
    {"Skills Auton", SkillsAuton, 61.5, 0, 90},
    {"Testing", testing, 0, 0, 0},
};
constexpr int autonCount = sizeof(autons) / sizeof(autons[0]);

int selectedAuton = 0; // starts on Driver
bool pickingAuton = true; // true from boot so nothing runs before the picker is done
bool autonLocked = false; // set once the route is locked in or started; selection stops changing
bool isRed = false; // alliance color; brain center button or controller Up toggles it

static const char* allianceName() { return isRed ? "RED" : "BLUE"; }

// Moves the selection, clamped to the list, and shows it on the brain. Called from
// the controller selector and the brain button callbacks, which run in different tasks.
void selectAuton(int index) {
    if (autonLocked) return;
    selectedAuton = std::clamp(index, 0, autonCount - 1);
    screenPrint(6, "Auton: %s  [%s]", autons[selectedAuton].name, allianceName());
}

void toggleAlliance() {
    if (autonLocked) return;
    isRed = !isRed;
    selectAuton(selectedAuton);
}

// Brain buttons: left/right step through the list, center toggles red/blue alliance.
// These only change the selection; the screen task does the drawing (LVGL isn't thread-safe).
static void brainLeftButton() { selectAuton(selectedAuton - 1); }
static void brainRightButton() { selectAuton(selectedAuton + 1); }
static void brainMiddleButton() { toggleAlliance(); }

void registerBrainAutonSelector() {
    pros::lcd::register_btn0_cb(brainLeftButton);
    pros::lcd::register_btn1_cb(brainMiddleButton);
    pros::lcd::register_btn2_cb(brainRightButton);
    selectAuton(selectedAuton); // show the starting choice
}

// Controller Left/Right change the selection and Up toggles red/blue alliance; line 2
// follows both (including changes from the brain buttons). The controller drops prints
// sent faster than ~50ms apart, so only print on a change, plus a slow refresh in case
// one got dropped. Line 2 shows "R " or "B " in front of the route name.
static void pollControllerSelector(int& shown, uint32_t& lastPrint) {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        selectAuton(selectedAuton - 1);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        selectAuton(selectedAuton + 1);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        toggleAlliance();
    }
    int state = selectedAuton * 2 + (isRed ? 1 : 0); // changes when route or alliance changes
    if (state != shown || pros::millis() - lastPrint > 250) {
        controller.print(2, 0, "%c %-13s", isRed ? 'R' : 'B', autons[selectedAuton].name);
        shown = state;
        lastPrint = pros::millis();
    }
}

void autonSelector() {
    pickingAuton = true; // screen task stops printing to controller line 2
    pros::delay(100);

    // step 1: pick the route, first X locks it in
    int shown = -1;
    uint32_t lastPrint = 0;
    while (true) {
        // on a competition switch/field, competition_initialize() and autonomous()
        // take over, so don't hold up initialize()
        if (pros::competition::is_connected()) {
            pickingAuton = false;
            return;
        }
        pollControllerSelector(shown, lastPrint);
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            break;
        }
        pros::delay(20);
    }

    autonLocked = true;
    AutonRoute& route = autons[selectedAuton];
    screenPrint(6, "Auton: %s [%s] (locked)", route.name, allianceName());

    // step 2: set the start pose, then wait for the second X to start.
    if (route.run) {
        chassis.setPose(route.x, route.y, route.theta);
        pros::delay(60);
        controller.print(2, 0, "X to start    ");
        uint32_t lastPrint = pros::millis();
        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            // plugged into a switch/field after locking in: autonomous() starts the route
            if (pros::competition::is_connected()) break;
            if (pros::millis() - lastPrint > 250) {
                controller.print(2, 0, "X to start    ");
                lastPrint = pros::millis();
            }
            pros::delay(20);
        }
    }

    pickingAuton = false;
}

void competitionAutonSelector() {
    int shown = -1;
    uint32_t lastPrint = 0;
    // PROS ends this task when the robot is enabled
    while (true) {
        pollControllerSelector(shown, lastPrint);
        pros::delay(20);
    }
}

bool autonRunning = false;

void runAuton() {
    autonRunning = true; // keeps the driver button task off the arm/claw
    if (autons[selectedAuton].run) autons[selectedAuton].run();
    autonRunning = false;
}

void runSelectedAuton() {
    AutonRoute& route = autons[selectedAuton];
    // if it was locked in with X, the pose was already set there and odom has tracked since
    if (!autonLocked && route.run) chassis.setPose(route.x, route.y, route.theta);
    autonLocked = true;
    runAuton();
}
