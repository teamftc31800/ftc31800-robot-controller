package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.ArmServo;
import org.firstinspires.ftc.teamcode.mechanisms.RGBIndicatorLight;

@TeleOp(name="TeamBotTeleop_latest", group="Drive")
public class TeamBotTeleop_latest extends OpMode {

    // Modular mechanisms
    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    private final Flywheel flywheel = new Flywheel();
    private final ArmServo armServo = new ArmServo();
    private final RGBIndicatorLight light = new RGBIndicatorLight();

    // Preset state tracking
    private boolean useManualPreset = false;
    private String activePresetLabel = "";

    // Shoot range debounce
    private int shootInRangeCount = 0;
    private static final int SHOOT_DEBOUNCE_FRAMES = 3;

    @Override
    public void init() {
        // Initialize all mechanisms
        drivetrain.init(hardwareMap, telemetry);
        intake.init(hardwareMap, telemetry);
        flywheel.init(hardwareMap, telemetry);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setTargetRPM(2400.0);
        flywheel.activate(); // Start flywheel active by default

        armServo.init(hardwareMap, telemetry);

        light.init(hardwareMap, telemetry, "indicator");
        light.blue();

        telemetry.addLine("Init complete.");
    }

    @Override
    public void loop() {
        // 1. DRIVETRAIN - Mecanum drive with gamepad1
        drivetrain.driveWithGamepad(
            gamepad1.left_stick_y,
            gamepad1.left_stick_x,
            gamepad1.right_stick_x
        );

        // 2. INTAKE - gamepad2 A/B/X
        intake.controlWithButtons(gamepad2.a, gamepad2.b, gamepad2.x);

        // 3. ARM SERVO - bumper control (one step per press)
        armServo.controlWithBumpers(gamepad2.right_bumper, gamepad2.left_bumper);

        // 4. PRESETS - RT and LT triggers
        handlePresets();

        // 5. FEEDER SERVOS - D-pad left/right
        flywheel.controlServosWithDpad(gamepad2.dpad_left, gamepad2.dpad_right);

        // 6. UPDATE FLYWHEEL
        flywheel.update();

        // 7. RGB INDICATOR - shoot range feedback
        updateShootRangeIndicator();

        // 8. TELEMETRY
        reportTelemetry();
    }

    private void handlePresets() {
        if (!armServo.isAvailable()) return;

        // RT preset: 150° / 2400 RPM
        if (gamepad2.right_trigger > 0.5) {
            armServo.setAngle(150.0);
            flywheel.setTargetRPM(2400.0);
            useManualPreset = true;
            activePresetLabel = "RT: 150°/2400 RPM";
        }

        // LT preset: 180° / 2200 RPM
        if (gamepad2.left_trigger > 0.5) {
            armServo.setAngle(180.0);
            flywheel.setTargetRPM(2200.0);
            useManualPreset = true;
            activePresetLabel = "LT: 180°/2200 RPM";
        }

        // X preset: 120° / 3150 RPM
        if (gamepad2.x) {
            armServo.setAngle(120.0);
            flywheel.setTargetRPM(3150.0);
            useManualPreset = true;
            activePresetLabel = "X: 120°/3150 RPM";
        }
    }

    private void updateShootRangeIndicator() {
        // Placeholder for AprilTag-based shoot range detection
        // When AprilTag is enabled, check if robot is in shooting zone
        boolean inShootRange = false; // Replace with aprilTagWebcam.IsRobotInZone(24) etc.

        if (inShootRange) {
            shootInRangeCount++;
        } else {
            shootInRangeCount = 0;
        }

        if (shootInRangeCount >= SHOOT_DEBOUNCE_FRAMES) {
            light.green();
        } else {
            light.blue();
        }
    }

    private void reportTelemetry() {
        telemetry.addLine("== DRIVE ==");
        telemetry.addLine("== INTAKE ==");

        // Flywheel telemetry
        flywheel.reportTelemetry();
        if (useManualPreset && !activePresetLabel.isEmpty()) {
            telemetry.addData("Preset", activePresetLabel);
        }

        // Arm servo telemetry
        armServo.reportTelemetry(useManualPreset ? activePresetLabel : null);

        // Hardware status
        telemetry.addLine("== HARDWARE STATUS ==");
        drivetrain.reportStatus();
        intake.reportStatus();
        flywheel.reportStatus();
        armServo.reportStatus();

        telemetry.update();
    }

    @Override
    public void start() {
        // Ensure flywheel starts spinning
        flywheel.activate();
    }
}
