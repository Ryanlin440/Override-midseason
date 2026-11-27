// Distance sensor offset tuner. Put the robot's center (the tracking center LemLib uses, not
// necessarily the middle of the frame) exactly at the pose you pass in, square to the walls. For the
// selected sensor it works out what offset would make the reading put the robot exactly there.
//
// Run it by calling offsetTuner(x, y, heading) from opcontrol() (like spinTest()), e.g.
// offsetTuner(0, 48, 90) with the robot on the y = 48 tile line facing the right wall. Pick a pose
// that puts each sensor you want to tune close to its wall (under ~30 in): the readings are much
// more accurate there. Controller:
//   A   next sensor
//   B   exit; prints the table with the new offsets to the terminal
//
// With the robot square, only the offset along the beam changes the answer (forward for front/back
// sensors, right for side sensors), so that's the one it calculates. Which wall each beam hits is
// worked out from the pose and the configured offsets, the same way distance_reset.cpp does it.
//
// The math is the same as distance_reset.cpp, run backwards: robot position + sensor offset +
// reading * beam direction = wall, solved for the offset instead of the robot's center. Starting
// values come from include/localization/robot_config.hpp. Copy the results into littleX() there;
// the UKF, distanceReset() and this tuner all read it.

#include <cmath>
#include <cstdio>

#include "localization/robot_config.hpp"
#include "robot.h"

bool tuningToolActive = false;

namespace {

constexpr int kAverageLoops = 50; // ~1 s of readings (the sensor refreshes every ~33 ms)
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double kMaxIncidenceDeg = 35.0; // same as MAX_INCIDENCE_DEG in distance_reset.cpp

struct Solution {
    bool valid = false;
    bool isForward = false; // which offset was calculated
    bool hitsXWall = false; // true: left/right wall, false: top/bottom wall
    double offset = 0;
    double expected = 0;    // reading the configured offsets predict, inches
    double incidence = 0;   // beam angle off square to the wall, degrees
};

// The offset that puts the robot's center exactly at (x, y) for reading d (inches) at heading
// headingDeg. Robot frame: +right, +forward; heading clockwise from +Y (LemLib).
//   left/right wall:  wall = x + (right*cos t + forward*sin t)   + d*dirX
//   top/bottom wall:  wall = y + (-right*sin t + forward*cos t)  + d*dirY
// Solved for whichever offset the heading makes count (the other one stays as configured).
Solution solveOffset(const loc::DistanceSensorConfig& s, double d, double x, double y, double headingDeg,
                     double fieldHalf) {
    Solution out;
    double t = headingDeg * DEG_TO_RAD;
    double c = std::cos(t), sn = std::sin(t);
    double a = t + s.angleDeg * DEG_TO_RAD;
    double dirX = std::sin(a), dirY = std::cos(a);

    // Which wall the beam hits, from the pose and the configured offsets.
    double sx = x + s.right * c + s.forward * sn;
    double sy = y - s.right * sn + s.forward * c;
    double tX = INFINITY, tY = INFINITY;
    if (dirX > 1e-9) tX = (fieldHalf - sx) / dirX;
    else if (dirX < -1e-9) tX = (-fieldHalf - sx) / dirX;
    if (dirY > 1e-9) tY = (fieldHalf - sy) / dirY;
    else if (dirY < -1e-9) tY = (-fieldHalf - sy) / dirY;
    out.hitsXWall = tX < tY;
    out.expected = out.hitsXWall ? tX : tY;
    out.incidence = std::acos(std::fmin(1.0, std::fabs(out.hitsXWall ? dirX : dirY))) / DEG_TO_RAD;
    if (!(out.expected > 0)) return out; // pose puts the sensor past the wall

    double rhs = out.hitsXWall ? (dirX > 0 ? fieldHalf : -fieldHalf) - d * dirX - x
                               : (dirY > 0 ? fieldHalf : -fieldHalf) - d * dirY - y;
    // rhs = (x wall)  right*c + forward*sn      (y wall)  -right*sn + forward*c
    double rightCoef = out.hitsXWall ? c : -sn;
    double forwardCoef = out.hitsXWall ? sn : c;
    if (std::fabs(forwardCoef) >= std::fabs(rightCoef)) {
        out.isForward = true;
        out.offset = (rhs - s.right * rightCoef) / forwardCoef;
    } else {
        out.isForward = false;
        out.offset = (rhs - s.forward * forwardCoef) / rightCoef;
    }
    out.valid = std::isfinite(out.offset);
    return out;
}

} // namespace

void offsetTuner(double x, double y, double headingDeg) {
    loc::RobotConfig cfg = loc::littleX();
    int count = (int)cfg.sensors.size();
    if (count == 0) return;

    headingDeg = std::fmod(headingDeg, 360.0);
    if (headingDeg < 0) headingDeg += 360.0;

    tuningToolActive = true; // pauses the controller pose line
    int sensor = 0;
    Solution best[16]; // last solution per sensor, for the table at the end

    double history[kAverageLoops] = {};
    int historyCount = 0, historyNext = 0;
    uint32_t lastPrint = 0;
    int printLine = 0;
    controller.clear();
    pros::delay(60);

    while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        uint32_t now = pros::millis();

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            sensor = (sensor + 1) % count;
            historyCount = historyNext = 0;
        }

        // Read and average the selected sensor.
        loc::DistanceSensorConfig& s = cfg.sensors[sensor];
        pros::Distance* device = distanceSensorFor(s.name);
        int mm = device ? device->get() : PROS_ERR;
        if (mm > 0 && mm < 9999 && mm != PROS_ERR) {
            history[historyNext] = mm / 25.4;
            historyNext = (historyNext + 1) % kAverageLoops;
            if (historyCount < kAverageLoops) historyCount++;
        }
        double d = -1;
        if (historyCount > 0) {
            d = 0;
            for (int i = 0; i < historyCount; i++) d += history[i];
            d /= historyCount;
        }

        Solution sol;
        if (d >= 0) sol = solveOffset(s, d, x, y, headingDeg, cfg.fieldHalf);
        bool steep = sol.incidence > kMaxIncidenceDeg;
        if (sol.valid && !steep && sensor < 16) best[sensor] = sol;
        double current = sol.isForward ? s.forward : s.right;
        // ! = past this sensor's maxRange, / = beam too far off square to trust
        const char* warn = steep ? "/" : d > s.maxRange ? "!" : " ";

        // The controller drops prints sent less than ~50 ms apart, so update one line at a time.
        if (now - lastPrint >= 60) {
            lastPrint = now;
            if (printLine == 0) {
                if (sol.valid) controller.print(0, 0, "%-5s %s %7.3f ", s.name, sol.isForward ? "fwd" : "rgt", sol.offset);
                else controller.print(0, 0, "%-5s no reading  ", s.name);
            } else if (printLine == 1) {
                controller.print(1, 0, "%s wall exp%6.2f ", sol.hitsXWall ? "x" : "y", sol.expected);
            } else {
                controller.print(2, 0, "d%6.2f was%6.2f%s", d, current, warn);
            }
            printLine = (printLine + 1) % 3;
        }
        if (sol.valid) {
            screenPrint(5, "TUNE %s: %s offset %.3f (was %.3f)", s.name, sol.isForward ? "forward" : "right",
                        sol.offset, current);
            screenPrint(6, "pose (%.1f, %.1f, %.0f) %s wall  read %.2f exp %.2f%s%s", x, y, headingDeg,
                        sol.hitsXWall ? "x" : "y", d, sol.expected, d > s.maxRange ? " PAST maxRange" : "",
                        steep ? " TOO ANGLED" : "");
        } else {
            screenPrint(5, "TUNE %s: no reading", s.name);
            screenPrint(6, "");
        }

        pros::delay(20);
    }

    std::printf("\n--- offsets from pose (%.2f, %.2f, %.1f): paste into littleX() in "
                "include/localization/robot_config.hpp ---\n", x, y, headingDeg);
    for (int i = 0; i < count; i++) {
        loc::DistanceSensorConfig& s = cfg.sensors[i];
        bool tuned = i < 16 && best[i].valid;
        if (tuned) (best[i].isForward ? s.forward : s.right) = best[i].offset;
        std::printf("  %-6s right %7.3f  forward %7.3f  angle %5.1f  %s\n", s.name, s.right, s.forward, s.angleDeg,
                    tuned ? (best[i].isForward ? "(forward tuned)" : "(right tuned)") : "(not tuned)");
    }
    screenPrint(5, "offset tuner done, table in terminal");
    screenPrint(6, "");
    controller.clear();
    tuningToolActive = false;
}
