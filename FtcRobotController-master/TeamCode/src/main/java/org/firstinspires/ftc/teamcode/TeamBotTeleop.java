package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.ConfigurationType;
//import com.qualcomm.robotcore.hardware.HardwareDevice;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.FeederLauncher;
import org.firstinspires.ftc.teamcode.mechanisms.RGBIndicatorLight;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@TeleOp(name="TeamBotTeleop", group="Drive")
public class TeamBotTeleop extends OpMode {

    // Drivetrain motors (312 rpm goBILDA)
    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft   = null;
    private DcMotor backRight  = null;

    // Intake (312 rpm goBILDA)
    private DcMotor intake = null;

    // Flywheel (high-speed shooter)
    private DcMotorEx flywheel = null;

    // Feeder motor (312 rpm goBILDA)
    private DcMotorEx feederMotor;

    // TWO feeder servos (dual feeder)
    private CRServo feederServoLeft  = null;
    private CRServo feederServoRight = null;

    private boolean hasFlywheel = false;
    private boolean hasFeederMotor = false;
    private boolean hasFeederServoLeft = false;
    private boolean hasFeederServoRight = false;

    private boolean flywheelActive = false;
    private boolean lastButtonState = false;

    // Convenience: did each motor initialize?
    private boolean hasFrontLeft  = false;
    private boolean hasFrontRight = false;
    private boolean hasBackLeft   = false;
    private boolean hasBackRight  = false;
    private boolean hasIntake     = false;
    private double largest_overshoot = 0;
    private static final double F                = 400.0;  // suggested feed speed
    private static final double FEEDER_HOLD_RPM  = 0.0;    // stop when not ready

    // Ticks per revolution (output shaft)
    // Adjust if your exact models differ.
    private static final double FLYWHEEL_TPR = 28.0;     // e.g., high-RPM 6000 motor often ~28 tpr at output (check your unit)
    private static final double FEEDER_TPR   = 537.7;    // goBILDA 312 rpm (19.2:1) ≈ 537.7 tpr

    private static final double FEEDER_RPM = 400;

    // Targets
    private static double FLYWHEEL_TARGET_RPM = 2750.0;  // your original target
    private static final double FLYWHEEL_TOLERANCE   = 200.0;  // +/- RPM window

    // Debounce: require N consecutive "in-tolerance" reads before feeding
    private int inToleranceCount = 0;

    private int dpad_pressed = 0;

    private static final int IN_TOLERANCE_REQUIRED = 1;

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    FeederLauncher leftFeederLauncher = new FeederLauncher();

    FeederLauncher rightFeederLauncher = new FeederLauncher();

    RGBIndicatorLight light = new RGBIndicatorLight();


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

        //flywheel   = getMotorEx("launcher"); // needs DcMotorEx for velocity

        hasFrontLeft  = (frontLeft  != null);
        hasFrontRight = (frontRight != null);
        hasBackLeft   = (backLeft   != null);
        hasBackRight  = (backRight  != null);
        hasIntake     = (intake     != null);
        //hasFlywheel   = (flywheel   != null);

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

//        if (hasFlywheel) {
//            // Shooter usually FLOATS on zero, so wheel can spin down naturally.
//            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//            // We’ll switch to RUN_USING_ENCODER so .setVelocity() works in loop()
//            flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            flywheel.setDirection(DcMotor.Direction.REVERSE);
//        }

        // Feeder motor
        try {
            feederMotor = hardwareMap.get(DcMotorEx.class, "feeder");
            feederMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            feederMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            feederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            hasFeederMotor = true;
        } catch (Exception e) {
            telemetry.addLine("⚠️ Feeder motor not found (name: 'feeder').");
        }

        // LEFT feeder servo
//        try {
//            feederServoLeft = hardwareMap.get(CRServo.class, "feederServoLeft");
//            feederServoLeft.setPower(0.0);
//            hasFeederServoLeft = true;
//        } catch (Exception e) {
//            telemetry.addLine("⚠️ Left feeder servo not found (name: 'feederServoLeft').");
//        }

        // RIGHT feeder servo
//        try {
//            feederServoRight = hardwareMap.get(CRServo.class, "feederServoRight");
//            feederServoRight.setPower(0.0);
//            hasFeederServoRight = true;
//        } catch (Exception e) {
//            telemetry.addLine("⚠️ Right feeder servo not found (name: 'feederServoRight').");
//        }

        //leftfeederlauncher
        leftFeederLauncher.init(hardwareMap,telemetry,"launcher","feederServoLeft");

        rightFeederLauncher.init(hardwareMap,telemetry,"null","feederServoRight");

        //AprilTag
        aprilTagWebcam.init(hardwareMap,telemetry);
        aprilTagWebcam.setRangeTolerance(5);
        aprilTagWebcam.setBearingTolerance(5);
        aprilTagWebcam.setTargetRange(45);
        aprilTagWebcam.setTargetBearing(5);

//
//        telemetry.addLine("Init complete. Check missing hardware below.");
//        reportHardwareStatus();
        aprilTagWebcam.reportHardwareStatus();
        leftFeederLauncher.reportHardwareStatus();
        rightFeederLauncher.reportHardwareStatus();

        light.init(hardwareMap,telemetry,"indicator");
        light.blue();

    }

    @Override
    public void loop() {


        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);
        aprilTagWebcam.displayDetectionTelemetry(id24);

        //Todo Blue vs Red alliance based checking based on gamepad selection.
        if (aprilTagWebcam.IsRobotinZone(24)) {
            telemetry.addLine("Robot in Zone to shoot");
            light.green();
        } else {
            light.off();
        }

        //----------------------------------
        // 1. DRIVE: mecanum with gamepad1
        //----------------------------------
        double y = -gamepad1.left_stick_y;  // forward = +1
        double x =  gamepad1.left_stick_x;  // strafe right = +1
        double rx = gamepad1.right_stick_x; // rotate right = +1

        // Basic mecanum math
        double flPower = y + x + rx;
        double blPower = y - x + rx;
        double frPower = y - x - rx;
        double brPower = y + x - rx;

        // Normalize so no value exceeds 1.0
        double max = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
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
        // 2. INTAKE CONTROL (gamepad2 A/B/X)
        //----------------------------------
        double intakePower = 0.0;
        if (gamepad2.a) {
            intakePower = 1.0;    // full forward
        } else if (gamepad2.b) {
            intakePower = -1.0;   // reverse / spit out
        } else if (gamepad2.x) {
            intakePower = 0.0;    // stop
        }
        if (hasIntake) {
            intake.setPower(intakePower);
        }

        //----------------------------------
        // 3. DUAL FEEDER SERVO CONTROL (D-PAD)
        //----------------------------------
        // Left servo: D-pad LEFT (forward)
        // Right servo: D-pad RIGHT (forward)
//        double leftServoPower  = 0.0;
//        double rightServoPower = 0.0;

        if (gamepad2.dpad_left) {
//            leftServoPower = -1.0;    // forward
            leftFeederLauncher.launch(0,0,true, true);
        }
        leftFeederLauncher.update();
        if (gamepad2.dpad_right) {
//            rightServoPower = -1.0;   // forward
            rightFeederLauncher.launch(0,0,true,true);
        }
        rightFeederLauncher.update();

//        if (hasFeederServoLeft) {
//            feederServoLeft.setPower(leftServoPower);
//        }
//        if (hasFeederServoRight) {
//            feederServoRight.setPower(rightServoPower);
//        }

//        telemetry.addData("Feeder Servo L Power", hasFeederServoLeft ? leftServoPower : 0.0);
//        telemetry.addData("Feeder Servo R Power", hasFeederServoRight ? rightServoPower : 0.0);

        //----------------------------------
        // 4. FLYWHEEL + FEEDER MOTOR SEQUENCE
        //----------------------------------
//        boolean toggleButton = gamepad2.triangle;    // use triangle to toggle sequence
//        boolean currentButtonState = toggleButton;
//
//        // Rising edge detect
//        if (currentButtonState && !lastButtonState) {
//            flywheelActive = !flywheelActive; // toggle state
//        }
//
//        // Save for next loop
//        lastButtonState = currentButtonState;
//
//        if (hasFlywheel) {
//            if (flywheelActive) {
////                if (gamepad2.dpad_up && (dpad_pressed == 0)) {
////                    FLYWHEEL_TARGET_RPM += 50;
////                    dpad_pressed += 1;
////                } else if (gamepad2.dpad_down && (dpad_pressed == 0)) {
////                    FLYWHEEL_TARGET_RPM -= 50;
////                    dpad_pressed += 1;
////                } else {
////                    dpad_pressed = 0;
////                }
//                // Spin up flywheel to target velocity
//                double targetTPS = rpmToTicksPerSec(FLYWHEEL_TARGET_RPM, FLYWHEEL_TPR);
//                flywheel.setVelocity(targetTPS);
//
//                // Read actual RPM
//                double currentTPS = flywheel.getVelocity(); // ticks/sec
//                double currentRPM = ticksPerSecToRPM(currentTPS, FLYWHEEL_TPR);
//                boolean atSpeed = Math.abs(currentRPM - FLYWHEEL_TARGET_RPM) <= FLYWHEEL_TOLERANCE;
//
//                double overshoot = currentRPM-FLYWHEEL_TARGET_RPM;
//                if (overshoot>largest_overshoot) {
//                    largest_overshoot = overshoot;
//                }
//                if (atSpeed) {
//                    inToleranceCount = Math.min(inToleranceCount + 1, IN_TOLERANCE_REQUIRED);
//                } else {
//                    inToleranceCount = 0;
//                }
//
//                // Run feeder motor only when flywheel is stably at speed
//                if (hasFeederMotor) {
//                    if (inToleranceCount >= IN_TOLERANCE_REQUIRED) {
//                        feederMotor.setVelocity(rpmToTicksPerSec(FEEDER_RPM, FEEDER_TPR));
//                    }
//                }
//
//                telemetry.addData("Flywheel", "Target %.0f RPM | Now %.0f RPM %s",
//                        FLYWHEEL_TARGET_RPM, currentRPM, atSpeed ? "(near target)" : "");
////                telemetry.addData("FeederMotor", (hasFeederMotor
////                        ? (inToleranceCount >= IN_TOLERANCE_REQUIRED ? "FEEDING" : "WAITING")
////                        : "N/A"));
//                telemetry.addData("Largest Overshoot", largest_overshoot);
//            } else {
//                // Toggle off: stop both
//                flywheel.setPower(0.0);
//                inToleranceCount = 0;
//
//                if (hasFeederMotor) {
//                    feederMotor.setPower(0.0);
//                }
////                telemetry.addData("Flywheel State", "Stopped");
////                telemetry.addData("FeederMotor", hasFeederMotor ? "Stopped" : "N/A");
//            }
//        }
//
//        //----------------------------------
//        // 5. TELEMETRY / DRIVER FEEDBACK
//        //----------------------------------
//        telemetry.addLine("== DRIVE ==");
////        telemetry.addData("y/x/rx", "%.2f %.2f %.2f", y, x, rx);
////        telemetry.addData("fl/br/bl/fr", "%.2f %.2f %.2f %.2f",
////                flPower, brPower, blPower, frPower);
//
//        telemetry.addLine("== INTAKE ==");
////        telemetry.addData("intakePower", "%.2f", intakePower);
////        telemetry.addData("intakePresent", hasIntake);
//
////        telemetry.addData("Flywheel Active", flywheelActive);
//
//        if (hasFlywheel) {
////            telemetry.addData("triangle", "toggle flywheel+feeder sequence");
//            double currentVel = flywheel.getVelocity(); // ticks/sec
////            telemetry.addData("flywheelVel(ticks/s)", "%.1f", currentVel);
//
//            double ticksPerRev = 28.0;
//            double currentRPM = ticksPerSecToRPM(currentVel, ticksPerRev);
//            telemetry.addData("flywheelVel(RPM est)", "%.0f", currentRPM);
//        }

        // Hardware health summary so drivers instantly see missing motors
        reportHardwareStatus();
        leftFeederLauncher.reportHardwareStatus();
        rightFeederLauncher.reportHardwareStatus();

        telemetry.update();
    }

    // -------------------------------------------------
    // Helper: safe motor fetch (DcMotor)
    // -------------------------------------------------
    private DcMotor getMotor(String name) {
        try {
            return hardwareMap.get(DcMotor.class, name);
        } catch (Exception e) {
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

//        telemetry.addData("flywheel",   hasFlywheel   ? "OK" : "MISSING");
        telemetry.addData("feederMotor",hasFeederMotor ? "OK" : "MISSING");
//        telemetry.addData("feederServoLeft",  hasFeederServoLeft  ? "OK" : "MISSING");
//        telemetry.addData("feederServoRight", hasFeederServoRight ? "OK" : "MISSING");
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

    @Override
    public void start() {
        //start running the flywheel of left and right feederlauncher at the start itself.
        //But not ready to shoot
        leftFeederLauncher.launch(0,0,false, false);
        rightFeederLauncher.launch(0,0,false, false);
    }

}
