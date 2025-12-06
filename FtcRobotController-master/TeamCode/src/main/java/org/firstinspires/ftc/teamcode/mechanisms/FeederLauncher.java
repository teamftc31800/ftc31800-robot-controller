package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeamBotTeleop;

public class FeederLauncher {

    private Telemetry telemetry;
    private CRServo feeder = null;

    private DcMotorEx flywheel = null;

    private final double FLYWHEEL_DEFAULT_VELOCITY = 1500; // start around mid-speed; tune with dpad

    private double targetFlywheelRPM = FLYWHEEL_DEFAULT_VELOCITY; // start around mid-speed; tune with dpad

    private boolean hasFlywheel = false;
    private boolean hasFeeder = false;

    private double targetDistance = 0;

    private double largest_overshoot = 0;

    final double ticksPerRev = 28.0; // *** TODO: PUT YOUR FLYWHEEL ENCODER TICKS/REV HERE ***

    final double FLYWHEEL_MIN_VELOCITY_PERCENT = 0.9; // 90% of target speed

    ElapsedTime feederTimer = new ElapsedTime();

    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    private enum LaunchState {
        IDLE,
        FEED,
        GET_READY,
        LAUNCHING,
        LAUNCHED
    }

    private double servoPower = 0.0;
    LaunchState launchState = LaunchState.IDLE;
    private boolean onceOveride = false;

    private boolean shoot = false;
    private static final double FLYWHEEL_TOLERANCE = 200.0;  // +/- RPM window

    // Debounce: require N consecutive "in-tolerance" reads before feeding
    private int inToleranceCount = 0;

    private static final int IN_TOLERANCE_REQUIRED = 1;
    
    private String flyWheelName;
    private String feederName;

    public void init(HardwareMap hwMap, Telemetry telemetry, String launcherName, String feederName) {

        this.telemetry = telemetry;
        this.flyWheelName = launcherName;
        this.feederName = feederName;

        try {
            flywheel = hwMap.get(DcMotorEx.class, launcherName);

        } catch (Exception e) {
            telemetry.addData("⚠️ Launcher not found", launcherName);
        }

        try {
            feeder = hwMap.get(CRServo.class, feederName);
        } catch (Exception e) {
            telemetry.addData("⚠️ Feeder not found", feederName);
        }


        hasFlywheel = (flywheel != null);
        hasFeeder = (feeder != null);

        if (hasFeeder) {
            feeder.setPower(0.0);
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


    }

    public void update(double targetRPM, double deltaRPM, boolean override, boolean feed) {
        if (!shoot && feed) {
            //this.targetDistance = distance;
            double targetTPS = 0;
            // compute target flywheel rpm based on distance.
            if (!override) {
                targetFlywheelRPM = targetRPM;
            }

            if (override && !onceOveride) {
                targetFlywheelRPM += deltaRPM;
                onceOveride = true;
            }

            //start the flywheel before feeder in the update itself.
            if (hasFlywheel) {
                targetTPS = rpmToTicksPerSec(targetFlywheelRPM, ticksPerRev);

                flywheel.setVelocity(targetTPS);
            }
            shoot = true;
            launchState = LaunchState.IDLE;
        }
    }

    public void Launch( ) {

        double targetTPS = 0;

        switch (launchState) {
            case IDLE:
                if (hasFlywheel) {

                    // Read actual RPM
                    double currentTPS = flywheel.getVelocity(); // ticks/sec
                    double currentRPM = ticksPerSecToRPM(currentTPS, ticksPerRev);
                    boolean atSpeed = Math.abs(currentRPM - targetFlywheelRPM) <= FLYWHEEL_TOLERANCE;

                    double overshoot = currentRPM - targetFlywheelRPM;
                    if (overshoot > largest_overshoot) {
                        largest_overshoot = overshoot;
                    }
                    if (atSpeed) {
                        inToleranceCount = Math.min(inToleranceCount + 1, IN_TOLERANCE_REQUIRED);
                        launchState = LaunchState.GET_READY;
                    } else {
                        inToleranceCount = 0;
                    }
                    telemetry.addData("Flywheel", "Target %.0f RPM | Now %.0f RPM %s",
                            targetFlywheelRPM, currentRPM, atSpeed ? "(near target)" : "");
                    telemetry.addData("Flywheel", "Target %.0f TPS | Now %.0f TPS %s",
                            targetTPS, currentTPS, atSpeed ? "(near target)" : "");
                    telemetry.addData("Largest Overshoot", largest_overshoot);

                }

                break;
            case LAUNCHING:
                servoPower = -1.0;
                if (hasFeeder && shoot) {
                    feeder.setPower(servoPower);
                    launchState = LaunchState.LAUNCHED;

                }
                break;
            case LAUNCHED:
                servoPower = 0;
                if (hasFeeder && shoot) {
                    feeder.setPower(servoPower);
                    shoot = false;
                    launchState = LaunchState.IDLE;
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

        telemetry.addData(flyWheelName, hasFlywheel ? "OK" : "MISSING");
        telemetry.addData(feederName, hasFeeder ? "OK" : "MISSING");

    }

    public void stop() {
        if (hasFlywheel) {
            flywheel.setPower(0);
        }
        if (hasFeeder) {
            feeder.setPower(0);
        }

    }

    //rotate backwards to release the artifact
    public void recoverFromStuck() {
        if (hasFlywheel) {
            flywheel.setPower(-1);
        }
        if (hasFeeder) {
            feeder.setPower(1);
        }
    }

}




