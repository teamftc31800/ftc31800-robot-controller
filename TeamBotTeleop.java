package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;

@TeleOp(name="TeamBotTeleop", group="Drive")
@Disabled
public class TeamBotTeleop extends OpMode {

    // Modular mechanisms
    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    private final Flywheel flywheel = new Flywheel();

    // Toggle state tracking
    private boolean lastToggleButton = false;

    @Override
    public void init() {
        // Initialize all mechanisms
        drivetrain.init(hardwareMap, telemetry);
        intake.init(hardwareMap, telemetry);
        flywheel.init(hardwareMap, telemetry);

        telemetry.addLine("Init complete. Hardware status:");
        reportHardwareStatus();
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

        // 3. FEEDER SERVOS - D-pad left/right
        flywheel.controlServosWithDpad(gamepad2.dpad_left, gamepad2.dpad_right);

        // 4. FLYWHEEL TOGGLE - triangle button
        boolean toggleButton = gamepad2.triangle;
        if (toggleButton && !lastToggleButton) {
            flywheel.toggle();
        }
        lastToggleButton = toggleButton;

        // 5. UPDATE FLYWHEEL (runs velocity control loop)
        if (flywheel.isActive()) {
            flywheel.update();
        }

        // 6. TELEMETRY
        telemetry.addLine("== DRIVE ==");
        telemetry.addLine("== INTAKE ==");

        if (flywheel.isActive()) {
            flywheel.reportTelemetry();
        } else {
            telemetry.addData("Flywheel", "Inactive (triangle to toggle)");
        }

        reportHardwareStatus();
        telemetry.update();
    }

    private void reportHardwareStatus() {
        telemetry.addLine("== HARDWARE STATUS ==");
        drivetrain.reportStatus();
        intake.reportStatus();
        flywheel.reportStatus();
    }
}
