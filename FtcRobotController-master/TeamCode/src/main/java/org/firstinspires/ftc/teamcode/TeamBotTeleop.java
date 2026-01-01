package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.FeederLauncher;
import org.firstinspires.ftc.teamcode.mechanisms.RGBIndicatorLight;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="TeamBotTeleop", group="Drive")
public class TeamBotTeleop extends OpMode {

    // -----------------------------
    // AprilTag + RGB (single instance)
    // -----------------------------
    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private final RGBIndicatorLight light = new RGBIndicatorLight();

    // Debounce for shoot-ready indicator
    private int shootInRangeCount = 0;
    private static final int SHOOT_DEBOUNCE_FRAMES = 3;

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
    private static final double FLYWHEEL_TPR = 28.0;
    private static final double FEEDER_TPR   = 537.7;

    private static final double FEEDER_RPM = 400;

    // Targets
    private static double FLYWHEEL_TARGET_RPM = 2750.0;
    private static final double FLYWHEEL_TOLERANCE   = 200.0;

    // Debounce: require N consecutive "in-tolerance" reads before feeding
    private int inToleranceCount = 0;
    private int dpad_pressed = 0;
    private static final int IN_TOLERANCE_REQUIRED = 1;

    // Launchers
    private final FeederLauncher leftFeederLauncher  = new FeederLauncher();
    private final FeederLauncher rightFeederLauncher = new FeederLauncher();

    @Override
    public void init() {
        // Motors
        frontLeft  = getMotor("left_front_drive");
        frontRight = getMotor("right_front_drive");
        backLeft   = getMotor("left_back_drive");
        backRight  = getMotor("right_back_drive");
        intake     = getMotor("intake");

        hasFrontLeft  = (frontLeft  != null);
        hasFrontRight = (frontRight != null);
        hasBackLeft   = (backLeft   != null);
        hasBackRight  = (backRight  != null);
        hasIntake     = (intake     != null);

        // Drivetrain directions
        if (hasFrontLeft)  frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (hasBackLeft)   backLeft.setDirection(DcMotor.Direction.REVERSE);
        if (hasFrontRight) frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (hasBackRight)  backRight.setDirection(DcMotor.Direction.FORWARD);





        // Brake
        if (hasFrontLeft)  frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasFrontRight) frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasBackLeft)   backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasBackRight)  backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasIntake)     intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        // Launchers
        leftFeederLauncher.init(hardwareMap, telemetry, "launcher", "feederServoLeft");
        rightFeederLauncher.init(hardwareMap, telemetry, "launcher", "feederServoRight");

        // AprilTag
        aprilTagWebcam.init(hardwareMap, telemetry);

        // ~1 foot (12 inches) shoot window
        aprilTagWebcam.setTargetRange(36);
        aprilTagWebcam.setRangeTolerance(5);      // 10–14 inches
        aprilTagWebcam.setTargetBearing(5);
        aprilTagWebcam.setBearingTolerance(5);

        // Status
        aprilTagWebcam.reportHardwareStatus();
        leftFeederLauncher.reportHardwareStatus();
        rightFeederLauncher.reportHardwareStatus();

        // RGB
        light.init(hardwareMap, telemetry, "indicator");
        light.blue();
    }

    @Override
    public void loop() {

        // -----------------------------
        // AprilTag Update + Shoot Ready
        // -----------------------------
        aprilTagWebcam.update();

        AprilTagDetection tag24 = aprilTagWebcam.getTagBySpecificId(24); // RED goal
        AprilTagDetection tag20 = aprilTagWebcam.getTagBySpecificId(20); // BLUE goal

        // Telemetry (optional but useful)
        if (tag24 != null) {
            telemetry.addLine("Seeing Tag 24 (RED GOAL)");
            telemetry.addData("Tag24 Range (in)", tag24.ftcPose.range);
            telemetry.addData("Tag24 Bearing (deg)", tag24.ftcPose.bearing);
        }
        if (tag20 != null) {
            telemetry.addLine("Seeing Tag 20 (BLUE GOAL)");
            telemetry.addData("Tag20 Range (in)", tag20.ftcPose.range);
            telemetry.addData("Tag20 Bearing (deg)", tag20.ftcPose.bearing);
        }

        // In-range if either goal tag is in the ~1ft zone
        boolean inShootRange =
                aprilTagWebcam.IsRobotinZone(24) ||
                aprilTagWebcam.IsRobotinZone(20);

        // Debounce so light doesn't flicker
        if (inShootRange) shootInRangeCount++;
        else shootInRangeCount = 0;

        if (shootInRangeCount >= SHOOT_DEBOUNCE_FRAMES) {
            telemetry.addLine("✅ SHOOT RANGE (~1 ft) (Tag 24 or 20)");
            light.green();
        } else {
            light.off();
        }

        //----------------------------------
        // 1. DRIVE: mecanum with gamepad1
        //----------------------------------
        double y  = -gamepad1.left_stick_y;   // forward = +1
        double x  =  gamepad1.left_stick_x;   // strafe right = +1
        double rx =  gamepad1.right_stick_x;  // rotate right = +1

        double flPower = y + x + rx;
        double blPower = y - x + rx;
        double frPower = y - x - rx;
        double brPower = y + x - rx;

        double max = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        flPower /= max;
        blPower /= max;
        frPower /= max;
        brPower /= max;

        if (hasFrontLeft)  frontLeft.setPower(flPower);
        if (hasBackLeft)   backLeft.setPower(blPower);
        if (hasFrontRight) frontRight.setPower(frPower);
        if (hasBackRight)  backRight.setPower(brPower);

        //----------------------------------
        // 2. INTAKE CONTROL (gamepad2 A/B/X)
        //----------------------------------
        double intakePower = 0.0;
        if (gamepad2.a) {
            intakePower = 1.0;
        } else if (gamepad2.b) {
            intakePower = -1.0;
        } else if (gamepad2.x) {
            intakePower = 0.0;
        }
        if (hasIntake) {
            intake.setPower(intakePower);
        }

        //----------------------------------
        // 3. DUAL FEEDER SERVO CONTROL (D-PAD)
        //----------------------------------
        if (gamepad2.dpad_left) {
            leftFeederLauncher.launch(0, 0, true, true);
        }
        leftFeederLauncher.update();

        if (gamepad2.dpad_right) {
            rightFeederLauncher.launch(0, 0, true, true);
        }
        rightFeederLauncher.update();

        // Hardware status
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
        telemetry.addData("frontLeft",   hasFrontLeft  ? "OK" : "MISSING");
        telemetry.addData("frontRight",  hasFrontRight ? "OK" : "MISSING");
        telemetry.addData("backLeft",    hasBackLeft   ? "OK" : "MISSING");
        telemetry.addData("backRight",   hasBackRight  ? "OK" : "MISSING");
        telemetry.addData("intake",      hasIntake     ? "OK" : "MISSING");
        telemetry.addData("feederMotor", hasFeederMotor ? "OK" : "MISSING");
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
        // Start flywheels on launchers (not ready to shoot)
        leftFeederLauncher.launch(0, 0, false, false);
        rightFeederLauncher.launch(0, 0, false, false);
    }
}
