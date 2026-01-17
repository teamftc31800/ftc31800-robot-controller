package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "FlywheelArmStepControl", group = "Control")
public class FlywheelArmStepControl extends LinearOpMode {

    // *** CHANGE based on your measurement ***
    private static final int TICKS_AT_90_DEG = 800;     // encoder ticks at 90 degrees
    private static final double ANGLE_STEP_DEG = 5.0;   // degrees per bumper click

    private static final double ARM_MOVE_POWER = 0.01;

    private DcMotorEx armMotor;

    // Stores the commanded angle, not read directly from encoder
    private double targetAngleDeg = 0.0;

    // For edge detection
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper  = false;

    @Override
    public void runOpMode() throws InterruptedException {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        // If encoder decreases when arm moves toward 90 deg, reverse the motor
        armMotor.setDirection(DcMotor.Direction.REVERSE); // or REVERSE, whichever makes "up" positive

        // Zero encoder at start (assume starting at 0 degrees)
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        targetAngleDeg = 0.0;
        int initialTargetTicks = angleToTicks(targetAngleDeg);
        armMotor.setTargetPosition(initialTargetTicks);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetAngleDeg = 0.0; // start at 0 deg

        telemetry.addLine("Right bumper: +step, Left bumper: -step");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper  = gamepad1.left_bumper;

            // Rising edge: right bumper just pressed -> increase angle
            if (rightBumper && !prevRightBumper) {
                targetAngleDeg += ANGLE_STEP_DEG;
            }

            // Rising edge: left bumper just pressed -> decrease angle
            if (leftBumper && !prevLeftBumper) {
                targetAngleDeg -= ANGLE_STEP_DEG;
            }

            // Clamp to [0, 90]
            if (targetAngleDeg < 0.0)  targetAngleDeg = 0.0;
            if (targetAngleDeg > 90.0) targetAngleDeg = 90.0;

            // Command motor to new target angle
            moveArmToAngleDeg(targetAngleDeg);

            // Save previous states for next loop
            prevRightBumper = rightBumper;
            prevLeftBumper  = leftBumper;

            telemetry.addData("Target Angle (deg)", targetAngleDeg);
            telemetry.addData("Encoder", armMotor.getCurrentPosition());
            telemetry.addData("Target Ticks", armMotor.getTargetPosition());
            telemetry.addData("Busy", armMotor.isBusy());
            telemetry.update();

            idle();
        }
    }

    /**
     * Converts desired angle to encoder ticks and commands RUN_TO_POSITION.
     */
    private void moveArmToAngleDeg(double angleDeg) {
        // Convert angle to encoder ticks (linear mapping between 0 and 90)
        int targetTicks = (int) Math.round((angleDeg / 90.0) * TICKS_AT_90_DEG);

        armMotor.setTargetPosition(targetTicks);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_MOVE_POWER);
    }

    /**
     * Convert desired angle (0..90) to encoder ticks using linear mapping.
     */
    private int angleToTicks(double angleDeg) {
        // Map 0..90 degrees -> 0..TICKS_AT_90_DEG
        return (int) Math.round((angleDeg / 90.0) * TICKS_AT_90_DEG);
    }
}