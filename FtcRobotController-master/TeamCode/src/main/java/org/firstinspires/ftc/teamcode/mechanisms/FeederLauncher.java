package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class FeederLauncher {

    private Telemetry telemetry;
    private CRServo feeder = null;    // Continuous rotation servo that pushes rings/pixels into shooter

    private DcMotorEx flywheel = null; // Shooter flywheel motor

    private final double FLYWHEEL_DEFAULT_VELOCITY = 2200; // Default target RPM for flywheel
    private double targetFlywheelRPM = FLYWHEEL_DEFAULT_VELOCITY; // Current commanded flywheel RPM

    private boolean hasFlywheel = false; // Used to avoid null pointer if hardware missing
    private boolean hasFeeder = false;   // Used to avoid null pointer if hardware missing

    private double targetDistance = 0; // (Unused) Placeholder for distance-based shooting

    private double largest_overshoot = 0; // Tracks biggest RPM overshoot while tuning

    final double ticksPerRev = 28.0; // <<< YOUR FLYWHEEL ENCODER TICKS/REV HERE

    final double FLYWHEEL_MIN_VELOCITY_PERCENT = 0.9; // 90% of target RPM threshold (unused)

    ElapsedTime feederTimer = new ElapsedTime(); // Timer used to control feeder firing window

    final double FEED_TIME_MILLI_SECONDS = 1000; // Time in milliseconds feeder activates per shot
    final double STOP_SPEED = 0.0;         // Command to stop the CR servo
    final double FULL_SPEED = 1.0;         // Max CR servo power

    // State machine for shot sequencing
    public enum LaunchState {
        IDLE,         // Not shooting
        SPIN_UP,      // Flywheel is ramping up to target speed
        LAUNCHING,    // Feeder pushing item into flywheel
        LAUNCHED      // Completed shot
    }

    private double servoPower = 0.0;
    LaunchState launchState = LaunchState.IDLE;

    private boolean shoot = false;         // Flags when a shot sequence begins
    private static final double FLYWHEEL_TOLERANCE = 50.01383194;  // +/- allowable RPM error

    // Require N consecutive speed readings to be in tolerance
    private int inToleranceCount = 0;
    private static final int IN_TOLERANCE_REQUIRED = 5;

    private String flyWheelName;
    private String feederName;

    public void init(HardwareMap hwMap, Telemetry telemetry, String launcherName, String feederName) {

        this.telemetry = telemetry;
        this.flyWheelName = launcherName;
        this.feederName = feederName;

        // Attempt to assign flywheel motor from configuration
        try {
            flywheel = hwMap.get(DcMotorEx.class, launcherName);
        } catch (Exception e) {
            telemetry.addData("⚠️ Launcher not found", launcherName);
        }

        // Attempt to assign feeder CR servo
        try {
            feeder = hwMap.get(CRServo.class, feederName);
        } catch (Exception e) {
            telemetry.addData("⚠️ Feeder not found", feederName);
        }

        // Set availability flags
        hasFlywheel = (flywheel != null);
        hasFeeder = (feeder != null);

        // Stop feeder initially
        if (hasFeeder) {
            feeder.setPower(0.0);
        }

        // Configure flywheel if present
        if (hasFlywheel) {
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Allows spin-down
            flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);       // Reset encoder
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);            // Required for setVelocity()
            flywheel.setDirection(DcMotor.Direction.FORWARD);               // Typical shooter direction
        }

        launchState = LaunchState.IDLE;

    }

    // Called when user requests a shot
    public void launch(double targetRPM, double deltaRPM, boolean override,boolean feed) {
        double targetTPS = 0;
        if (!shoot && feed) { // Start sequence only if not already shooting
            inToleranceCount = 0;
            // If overriding, use the direct requested RPM

            // If overriding, apply deltaRPM ONCE
            if (override) {
                if (targetRPM > 0 ) {
                    targetFlywheelRPM = targetRPM;
                }
                targetFlywheelRPM += deltaRPM;
            }

            // Convert RPM to ticks/sec and set flywheel velocity
            if (hasFlywheel) {
                targetTPS = rpmToTicksPerSec(targetFlywheelRPM, ticksPerRev);
                flywheel.setVelocity(targetTPS);
            }

            shoot = true;                       // Signal that shot process has begun
            launchState = LaunchState.SPIN_UP;  // Begin flywheel ramp-up
        }
        else {
            if (hasFlywheel) {
                targetTPS = rpmToTicksPerSec(targetFlywheelRPM, ticksPerRev);
                flywheel.setVelocity(targetTPS);
            }
        }
    }

    public void update() {

        double targetTPS = 0;

        switch (launchState) {
            case IDLE:
                break;

            case SPIN_UP:
                if (hasFlywheel) {
                    // Get actual flywheel RPM
                    double currentTPS = flywheel.getVelocity();
                    double currentRPM = ticksPerSecToRPM(currentTPS, ticksPerRev);

                    boolean atSpeed =
                            Math.abs(currentRPM - targetFlywheelRPM) <= FLYWHEEL_TOLERANCE;

                    // Track maximum overshoot for tuning
                    double overshoot = currentRPM - targetFlywheelRPM;
                    if (overshoot > largest_overshoot) {
                        largest_overshoot = overshoot;
                    }

                    // If speed reached, move to launching
                    if (atSpeed && shoot) {
                        inToleranceCount = Math.min(inToleranceCount + 1, IN_TOLERANCE_REQUIRED);
                    } else {
                        inToleranceCount = 0;
                    }
                    if ((inToleranceCount == IN_TOLERANCE_REQUIRED) && shoot) {
                        launchState = LaunchState.LAUNCHING;
                    }

                    telemetry.addData("InTolerance Count: ", inToleranceCount);

                    // Telemetry for tuning
                    telemetry.addData("Flywheel", "Target %.0f RPM | Now %.0f RPM %s",
                            targetFlywheelRPM, currentRPM, atSpeed ? "(near target)" : "");
                    telemetry.addData("Flywheel", "Target %.0f TPS | Now %.0f TPS %s",
                            targetTPS, currentTPS, atSpeed ? "(near target)" : "");
                    telemetry.addData("Largest Overshoot", largest_overshoot);
                }
                break;

            case LAUNCHING:
                servoPower = -1.0; // Push item into flywheel (direction depends on servo mounting)

                if (hasFeeder) {
                    if (shoot) {
                        feeder.setPower(servoPower);      // Activate feeder
                        feederTimer.reset();
                        shoot = false;
                    } else {
                        //rotate feeder enough to release the artifact. might require tuning
                        if (feederTimer.milliseconds() > FEED_TIME_MILLI_SECONDS) {
                            launchState = LaunchState.LAUNCHED; // Move to next state
                        }
                    }
                }
                break;

            case LAUNCHED:
                servoPower = 0; // Stop feeder
                if (hasFeeder) {
                    feeder.setPower(servoPower);
                    launchState = LaunchState.SPIN_UP; // Keep wheel spinning for next shot
                }
                break;
        }


    }

    // -------------------------------------------------
    // Math helpers for flywheel RPM <-> ticks/sec
    // -------------------------------------------------
    private double rpmToTicksPerSec(double rpm, double ticksPerRev) {
        double ticksPerMinute = rpm * ticksPerRev;
        return ticksPerMinute / 60.0;
    }

    private double ticksPerSecToRPM(double ticksPerSec, double ticksPerRev) {
        double ticksPerMin = ticksPerSec * 60.0;
        return ticksPerMin / ticksPerRev;
    }

    public void reportHardwareStatus() {
        telemetry.addData(flyWheelName, hasFlywheel ? "OK" : "MISSING"); // Report flywheel status
        telemetry.addData(feederName, hasFeeder ? "OK" : "MISSING");     // Report feeder status
    }

    public void stop() {
        if (hasFlywheel) {
            flywheel.setPower(0); // Stop flywheel motor
        }
        if (hasFeeder) {
            feeder.setPower(0);   // Stop feeder
        }
        launchState = LaunchState.IDLE; // Reset state machine
    }

    // Rotate backwards to clear jams ("reverse shooter")
    public void recoverFromStuck() {
        if (hasFlywheel) {
            flywheel.setPower(-1);
        }
        if (hasFeeder) {
            feeder.setPower(1);
        }
        launchState = LaunchState.IDLE; // Reset state machine
        shoot = false;
    }

    public LaunchState getStatus() {
        return launchState;
    }
}
