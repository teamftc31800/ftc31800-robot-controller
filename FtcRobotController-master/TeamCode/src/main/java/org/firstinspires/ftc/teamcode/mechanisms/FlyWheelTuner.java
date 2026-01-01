package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlyWheelTuner {
    public DcMotorEx flywheelMotor;

    double highVelocity = 2850;

    public double lowVelocity = 2700;

    double curTargetVelocity = highVelocity;

    private static final double FLYWHEEL_TPR = 28.0;

    private Gamepad gamepad1 = null;

    double F = 16.2;
    double P = 20.9;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;

    Telemetry telemetry;

    public DcMotorEx init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad){
        this.gamepad1 = gamepad;

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        this.telemetry = telemetry;
        telemetry.addLine("Init complete");

        return flywheelMotor;
    }

    public void update(){

        if(gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            }else { curTargetVelocity = highVelocity; }

        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if(gamepad1.dpadRightWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if(gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];

        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        //set new PIDF coeifficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //set velocity
        double targetTPS = rpmToTicksPerSec(curTargetVelocity, FLYWHEEL_TPR);
        flywheelMotor.setVelocity((targetTPS));

        double curTPS = flywheelMotor.getVelocity();
        double curVelocity = ticksPerSecToRPM(curTPS,FLYWHEEL_TPR);
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/O)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);

    }

    private double rpmToTicksPerSec(double rpm, double ticksPerRev) {
        double ticksPerMinute = rpm * ticksPerRev;
        return ticksPerMinute / 60.0;
    }

    private double ticksPerSecToRPM(double ticksPerSec, double ticksPerRev) {
        double ticksPerMin = ticksPerSec * 60.0;
        return ticksPerMin / ticksPerRev;
    }
}

