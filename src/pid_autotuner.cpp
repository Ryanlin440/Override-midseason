/**
 * PID auto tuner (Ziegler-Nichols relay method).
 *
 * The PIDAutotuner class is ported from jackw01/arduino-pid-autotuner:
 *   https://github.com/jackw01/arduino-pid-autotuner
 *   Copyright (c) 2016-2020 jackw01, MIT License.
 *
 * How it works: the tuner drives the mechanism at full +output until it passes the
 * target, then full -output until it drops back below, over and over. From how far
 * it swings (amplitude) and how long one swing takes (period), Ziegler-Nichols gives
 * kP, kI and kD.
 *
 * The gains come out in the same units lemlib::PID uses: output is motor power
 * (-127..127), error is in the sensor's units (inches or degrees), and kI/kD are per
 * loop step. So they only fit a PID that runs every LOOP_MS (10 ms), which is what
 * LemLib's motion loops and moveCascadeTo() use.
 *
 * Usage: call one of the tune*() routines from opcontrol() before its while loop,
 * the same way as spinTest(). Each one blocks until done (or B is pressed), then
 * shows kP/kI/kD on brain lines 5-7 and prints them to the PROS terminal.
 * Treat the result as a starting point and fine-tune by hand from there.
 */

#include "main.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {

// -------------------------------------------------------- PIDAutotuner (ported)

class PIDAutotuner {
  public:
    enum ZNMode {
        ZNModeBasicPID, // fastest, most overshoot
        ZNModeLessOvershoot,
        ZNModeNoOvershoot // slowest, safest (default)
    };

    void setTargetInputValue(float target) { targetInputValue = target; }
    /// PID loop interval in microseconds; must match the loop the gains are for
    void setLoopInterval(long interval) { loopInterval = interval; }
    void setOutputRange(float min, float max) {
        minOutput = min;
        maxOutput = max;
    }
    void setZNMode(ZNMode zn) { znMode = zn; }
    void setTuningCycles(int tuneCycles) { cycles = tuneCycles; }

    /// Call immediately before the tuning loop starts, with the current time in us.
    void startTuningLoop(unsigned long us) {
        i = 0;
        output = true;
        outputValue = maxOutput;
        t1 = t2 = us;
        tHigh = tLow = 0;
        max = -1e12f;
        min = 1e12f;
        pAverage = iAverage = dAverage = 0;
        averaged = 0;
    }

    /// Run once per loop (at loopInterval) with the sensor reading and time in us.
    /// Returns the output to apply; don't change it or the results will be wrong.
    float tunePID(float input, unsigned long us) {
        max = (max > input) ? max : input;
        min = (min < input) ? min : input;

        // output on and input has risen past the target: turn output off
        if (output && input > targetInputValue) {
            output = false;
            outputValue = minOutput;
            t1 = us;
            tHigh = t1 - t2;
            max = targetInputValue;
        }

        // output off and input has dropped below the target: turn output on
        if (!output && input < targetInputValue) {
            output = true;
            outputValue = maxOutput;
            t2 = us;
            tLow = t2 - t1;

            // ultimate gain Ku = 4d / (pi * a): d = output amplitude, a = input amplitude
            float ku = (4.0f * ((maxOutput - minOutput) / 2.0f)) / (M_PI * (max - min) / 2.0f);
            // period of oscillation, in microseconds
            float tu = tLow + tHigh;

            float kpConstant, tiConstant, tdConstant;
            if (znMode == ZNModeBasicPID) {
                kpConstant = 0.6f;
                tiConstant = 0.5f;
                tdConstant = 0.125f;
            } else if (znMode == ZNModeLessOvershoot) {
                kpConstant = 0.33f;
                tiConstant = 0.5f;
                tdConstant = 0.33f;
            } else {
                kpConstant = 0.2f;
                tiConstant = 0.5f;
                tdConstant = 0.33f;
            }

            kp = kpConstant * ku;
            ki = (kp / (tiConstant * tu)) * loopInterval;
            kd = (tdConstant * kp * tu) / loopInterval;

            // skip the first two cycles (still settling), average the rest
            if (i > 1) {
                pAverage += kp;
                iAverage += ki;
                dAverage += kd;
                averaged++;
            }

            min = targetInputValue;
            i++;
        }

        if (i >= cycles) {
            output = false;
            outputValue = 0; // stop the mechanism (original used minOutput)
            // The original divided by (i - 1), but only i - 2 cycles get summed above,
            // which made every gain a bit low. Divide by the actual count instead.
            if (averaged > 0) {
                kp = pAverage / averaged;
                ki = iAverage / averaged;
                kd = dAverage / averaged;
            }
        }

        return outputValue;
    }

    float getKp() const { return kp; }
    float getKi() const { return ki; }
    float getKd() const { return kd; }
    bool isFinished() const { return i >= cycles; }
    int getCycle() const { return i; }

  private:
    float targetInputValue = 0;
    float loopInterval = 0;
    float minOutput = -127, maxOutput = 127;
    ZNMode znMode = ZNModeNoOvershoot;
    int cycles = 10;

    int i = 0;
    int averaged = 0;
    bool output = true;
    float outputValue = 0;
    long t1 = 0, t2 = 0, tHigh = 0, tLow = 0;
    float max = 0, min = 0;
    float pAverage = 0, iAverage = 0, dAverage = 0;
    float kp = 0, ki = 0, kd = 0;
};

// ------------------------------------------------------------ robot glue

constexpr int LOOP_MS = 10; // LemLib motion loops and moveCascadeTo() run every 10 ms

struct TuneResult {
    bool finished; // false if aborted with B or timed out
    float kp, ki, kd;
};

// Runs the relay loop: readInput() gives the sensor value, applyOutput() drives the
// mechanism. Press B to abort. Always leaves the mechanism stopped.
template <typename ReadFn, typename ApplyFn>
TuneResult runTuner(const char* name, float target, float maxPower, int cycles, int timeoutMs,
                    ReadFn readInput, ApplyFn applyOutput) {
    PIDAutotuner tuner;
    tuner.setTargetInputValue(target);
    tuner.setLoopInterval(LOOP_MS * 1000);
    tuner.setOutputRange(-maxPower, maxPower);
    tuner.setZNMode(PIDAutotuner::ZNModeNoOvershoot);//less overshoot or basic pid for more powerful
    tuner.setTuningCycles(cycles);

    screenPrint(5, "Tuning %s  B=abort", name);
    screenPrint(6, "");
    screenPrint(7, "");

    uint32_t start = pros::millis();
    uint32_t wake = start;
    bool aborted = false;
    tuner.startTuningLoop(pros::micros());
    while (!tuner.isFinished()) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) ||
            pros::millis() - start > (uint32_t)timeoutMs) {
            aborted = true;
            break;
        }
        float input = readInput();
        applyOutput(tuner.tunePID(input, pros::micros()));
        if ((pros::millis() - start) % 200 < LOOP_MS) {
            screenPrint(6, "Cycle %d/%d  in: %.2f", tuner.getCycle(), cycles, input);
        }
        pros::Task::delay_until(&wake, LOOP_MS); // keeps the loop at exactly LOOP_MS
    }
    applyOutput(0);

    if (aborted) {
        screenPrint(5, "%s tune aborted", name);
        printf("[autotune] %s aborted at cycle %d\n", name, tuner.getCycle());
        return {false, 0, 0, 0};
    }
    screenPrint(5, "%s tuned:", name);
    screenPrint(6, "kP %.3f  kI %.5f", tuner.getKp(), tuner.getKi());
    screenPrint(7, "kD %.3f", tuner.getKd());
    printf("[autotune] %s: kP=%.4f kI=%.6f kD=%.4f\n", name, tuner.getKp(), tuner.getKi(), tuner.getKd());
    return {true, tuner.getKp(), tuner.getKi(), tuner.getKd()};
}

} // namespace

// Cascade lift. Oscillates around targetInches, so pick a height in the middle of
// its travel where swinging a bit either way won't hit the top or bottom.
// Result goes in cascade_pid (src/main.cpp).
void tuneCascadePID(float targetInches, float maxPower) {
    bool wasRunning = autonRunning;
    autonRunning = true; // keep the driver button task off the cascade/claw
    cascade.set_brake_mode(pros::MotorBrake::coast);
    runTuner("Cascade", targetInches, maxPower, 10, 15000,
             [] { return getCascadeInches(); },
             [](float out) { cascade.move(out); });
    cascade.set_brake_mode(pros::MotorBrake::hold);
    cascade.move_velocity(0);
    autonRunning = wasRunning;
}

// Turning in place. Rocks back and forth around targetDeg from a 0 start heading.
// Result goes in angular_controller (src/main.cpp).
void tuneAngularPID(float targetDeg, float maxPower) {
    chassis.setPose(0, 0, 0);
    runTuner("Angular", targetDeg, maxPower, 10, 20000,
             [] { return chassis.getPose().theta; },
             [](float out) {
                 left_motors.move(out); // positive = clockwise = heading goes up
                 right_motors.move(-out);
             });
    left_motors.brake();
    right_motors.brake();
}

// Driving straight. Drives back and forth around targetInches ahead of where it
// starts, so leave room in front and behind. Result goes in lateral_controller.
void tuneLateralPID(float targetInches, float maxPower) {
    chassis.setPose(0, 0, 0);
    runTuner("Lateral", targetInches, maxPower, 10, 20000,
             [] { return chassis.getPose().y; },
             [](float out) {
                 left_motors.move(out);
                 right_motors.move(out);
             });
    left_motors.brake();
    right_motors.brake();
}
