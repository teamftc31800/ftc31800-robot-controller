

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.ConfigurationType;
//import com.qualcomm.robotcore.hardware.HardwareDevice;

import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeamBotTeleop", group="Drive")
public class TeamBotTeleop extends OpMode {

    // Drivetrain motors (312 rpm goBILDA)
    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft   = null;
    private DcMotor backRight  = null;

    // Intake (312 rpm goBILDA)
    private DcMotor intake = null;

    // Intake (312 rpm goBILDA)
    private DcMotor feeder = null;


    // Flywheel (high-speed shooter)
    private DcMotorEx flywheel = null;

    // Flywheel target RPM (we’ll let you tune live)
    private double targetFlywheelRPM = 1500; // start around mid-speed; tune with dpad

    // Convenience: did each motor initialize?
    private boolean hasFrontLeft  = false;
    private boolean hasFrontRight = false;
    private boolean hasBackLeft   = false;
    private boolean hasBackRight  = false;
    private boolean hasIntake     = false;
    private boolean hasFlywheel   = false;
    private boolean hasFeeder     = false;

    final double ticksPerRev = 28.0; // *** TODO: PUT YOUR FLYWHEEL ENCODER TICKS/REV HERE ***

    final double FLYWHEEL_MIN_VELOCITY_PERCENT = 0.9; // 90% of target speed

    ElapsedTime feederTimer = new ElapsedTime();

    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    @Override
    public void init() {
        /*
         * Try to grab each motor safely. If it's missing from the config,
         * we catch the exception and record a flag so we don't try to drive it later.
         */
        frontLeft  = getMotor("left_front_drive");
        frontRight = getMotor("right_front_drive");
        backLeft   = getMotor("left_back_drive");
        backRight  = getMotor("right_back_drive");
        intake     = getMotor("intake");
        feeder     = getMotor("feeder");
        flywheel   = getMotorEx("launcher"); // needs DcMotorEx for velocity

        hasFrontLeft  = (frontLeft  != null);
        hasFrontRight = (frontRight != null);
        hasBackLeft   = (backLeft   != null);
        hasBackRight  = (backRight  != null);
        hasIntake     = (intake     != null);
        hasFlywheel   = (flywheel   != null);
        hasFeeder     = (feeder     != null);

        // Reverse directions so forward stick actually drives forward.
        // Common mecanum setup: left side reversed, right side normal.
        if (hasFrontLeft)  frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (hasBackLeft)   backLeft.setDirection(DcMotor.Direction.REVERSE);
        if (hasFrontRight) frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (hasBackRight)  backRight.setDirection(DcMotor.Direction.FORWARD);

        // Zero power behavior so robot stops instead of coasting
        if (hasFrontLeft)  frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasFrontRight) frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasBackLeft)   backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasBackRight)  backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (hasIntake) {
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (hasFeeder) {
            feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (hasFlywheel) {
            // Shooter usually FLOATS on zero, so wheel can spin down naturally.
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // Optional: run without encoder for now, we’ll manually set velocity later.
            // We’ll switch to RUN_USING_ENCODER so .setVelocity() works in loop()
            flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setDirection(DcMotor.Direction.REVERSE);
        }

        telemetry.addLine("Init complete. Check missing hardware below.");
        reportHardwareStatus();
    }

    @Override
    public void loop() {
        //----------------------------------
        // 1. DRIVE: mecanum with gamepad1
        //----------------------------------
        // left_stick_y is typically forward/back, but it's inverted (up is -1).
        double y = -gamepad1.left_stick_y;  // forward = +1
        double x =  gamepad1.left_stick_x;  // strafe right = +1
        double rx = gamepad1.right_stick_x; // rotate right = +1

        // Basic mecanum math
        double flPower = y + x + rx;
        double blPower = y - x + rx;
        double frPower = y - x - rx;
        double brPower = y + x - rx;

        // Normalize so no value exceeds 1.0
        double max = Math.max(1.0,
                Math.max(Math.abs(flPower),
                        Math.max(Math.abs(blPower),
                                Math.max(Math.abs(frPower),
                                        Math.abs(brPower)))));

        flPower /= max;
        blPower /= max;
        frPower /= max;
        brPower /= max;

        // Send power only if that motor exists
        if (hasFrontLeft)  frontLeft.setPower(flPower);
        if (hasBackLeft)   backLeft.setPower(blPower);
        if (hasFrontRight) frontRight.setPower(frPower);
        if (hasBackRight)  backRight.setPower(brPower);

        //----------------------------------
        // 2. INTAKE CONTROL (gamepad1 A/B/X)
        //----------------------------------
        //  A = intake forward
        //  B = intake reverse
        //  X = stop intake
        double intakePower = 0.0;
        if (gamepad2.a) {
            intakePower = 1.0;    // full forward
        } else if (gamepad2.b) {
            intakePower = -1.0;   // reverse / spit out
        } else if (gamepad2.x) {
            intakePower = 0.0;
        }
        if (hasIntake) {
            intake.setPower(intakePower);
        }

        //----------------------------------
        // 3. FLYWHEEL CONTROL (gamepad1)
        //----------------------------------
        //  Right bumper  = enable flywheel at target RPM
        //  Left bumper   = stop flywheel
        //  Dpad up/down  = tune target RPM in steps
        //
        // We'll convert RPM -> ticksPerSecond and call setVelocity().
        //
        // NOTE: You *must* update "TICKS_PER_REV" below to match your flywheel motor encoder.
        // goBILDA 1172rpm motors often ~28 ticks per rev. 312rpm gobilda motor ~537.6 tpr.
        // Your ~6000rpm motor may be a coreless or planetary w/ custom encoder or no encoder.
        // If it has no encoder, setVelocity won't really work; you'd fall back to simple setPower().

        // Live adjust RPM setpoint:
        if (gamepad2.dpad_up) {
            targetFlywheelRPM += 50; // +100 RPM
        } else if (gamepad2.dpad_down) {
            targetFlywheelRPM -= 50; // -100 RPM
        }
        targetFlywheelRPM = Range.clip(targetFlywheelRPM, 500, 6000); // keep sane

        // Spin logic L1: Left bumper, R1: Right bumper
        boolean flywheelRequestedOn = gamepad2.right_bumper;
        boolean flywheelRequestedOff = gamepad2.left_bumper;

        if (hasFlywheel && hasFeeder) {
            launch(flywheelRequestedOn, flywheelRequestedOff);
        }

        //----------------------------------
        // 4. TELEMETRY / DRIVER FEEDBACK
        //----------------------------------
        telemetry.addLine("== DRIVE ==");
        telemetry.addData("fl/br/bl/fr", "%.2f %.2f %.2f %.2f",
                flPower, brPower, blPower, frPower);

        telemetry.addLine("== INTAKE ==");
        telemetry.addData("intakePower", "%.2f", intakePower);
        telemetry.addData("intakePresent", hasIntake);

        telemetry.addLine("== FLYWHEEL ==");
        telemetry.addData("targetRPM", "%.0f", targetFlywheelRPM);
        telemetry.addData("flywheelPresent", hasFlywheel);

        if (hasFlywheel) {
            telemetry.addData("right_bumper", "spin @ target RPM");
            telemetry.addData("left_bumper",  "stop");
            telemetry.addData("dpad up/down", "RPM +/- 100");

            // If encoder exists, show current velocity estimate
            double currentVel = flywheel.getVelocity(); // ticks/sec
            telemetry.addData("flywheelVel(ticks/s)", "%.1f", currentVel);
            
            
            double currentRPM = ticksPerSecToRPM(currentVel, ticksPerRev);
            telemetry.addData("flywheelVel(RPM est)", "%.0f", currentRPM);
        }

        // Hardware health summary so drivers instantly see missing motors
        reportHardwareStatus();

        telemetry.update();
    }

    // -------------------------------------------------
    // Helper: safe motor fetch (DcMotor)
    // -------------------------------------------------
    private DcMotor getMotor(String name) {
        try {
            return hardwareMap.get(DcMotor.class, name);
        } catch (Exception e) {
            // Could be not found in config or wrong type
            return null;
        }
    }

    // -------------------------------------------------
    // Helper: safe motor fetch as DcMotorEx
    // -------------------------------------------------
    private DcMotorEx getMotorEx(String name) {
        try {
            return hardwareMap.get(DcMotorEx.class, name);
        } catch (Exception e) {
            return null;
        }
    }

    // -------------------------------------------------
    // Helper: report which motors are missing
    // -------------------------------------------------
    private void reportHardwareStatus() {
        telemetry.addLine("== HARDWARE STATUS ==");

        telemetry.addData("frontLeft",  hasFrontLeft  ? "OK" : "MISSING");
        telemetry.addData("frontRight", hasFrontRight ? "OK" : "MISSING");
        telemetry.addData("backLeft",   hasBackLeft   ? "OK" : "MISSING");
        telemetry.addData("backRight",  hasBackRight  ? "OK" : "MISSING");

        telemetry.addData("intake",     hasIntake     ? "OK" : "MISSING");

        telemetry.addData("flywheel",   hasFlywheel   ? "OK" : "MISSING");
        telemetry.addData("feeder",     hasFeeder     ? "OK" : "MISSING");
    }

    // -------------------------------------------------
    // Math helpers for flywheel RPM <-> ticks/sec
    // -------------------------------------------------
    private double rpmToTicksPerSec(double rpm, double ticksPerRev) {
        // rpm * (ticks/rev) / 60 sec/min
        // Step by step to avoid mistakes:
        // 1. rpm * ticksPerRev = ticks per minute
        double ticksPerMinute = rpm * ticksPerRev;
        // 2. ticks per minute / 60 = ticks per second
        return ticksPerMinute / 60.0;
    }

    private double ticksPerSecToRPM(double ticksPerSec, double ticksPerRev) {
        // ticks/sec * 60 sec/min / ticksPerRev
        double ticksPerMin = ticksPerSec * 60.0;
        return ticksPerMin / ticksPerRev;
    }

    void launch(boolean shotRequested) {
        if (shotRequested) {
                
        } else if (flywheelRequestedOff) {
            flywheel.setPower(0);
        }
    }

    void launch(boolean shotRequested, boolean stopRequested) {
        
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                            // Convert RPM to ticks/sec and set velocity
                double targetTicksPerSec = rpmToTicksPerSec(targetFlywheelRPM, ticksPerRev);
                double minTicksPerSec = FLYWHEEL_MIN_VELOCITY_PERCENT * targetTicksPerSec;

                // Ask motor to run at that velocity.
                // The SDK will do PIDF internally in RUN_USING_ENCODER mode.
                
                flywheel.setVelocity(targetTicksPerSec);

                // Check if we've reached minimum speed to launch
                if (flywheel.getVelocity() > minTicksPerSec) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                // Activate feeder to push a ring into the flywheel
                feeder.setPower(FULL_SPEED);     
                // Start timer to time the feeding duration       
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                // Keep feeder running for set time
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    // Stop feeder
                    feeder.setPower(STOP_SPEED);
                    // stop flywheel too
                    flywheel.setPower(STOP_SPEED);                    
                }
                break;
        }
    }
}