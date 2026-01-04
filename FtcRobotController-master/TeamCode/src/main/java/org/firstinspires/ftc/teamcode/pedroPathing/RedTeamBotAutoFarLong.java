
package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.FeederLauncher;

@Autonomous(name = "RedTeamBotAutoFarLong", group = "Examples")
public class RedTeamBotAutoFarLong extends OpMode {

    private Follower follower;
    private ElapsedTime pathTimer, actionTimer, opmodeTimer, intakeTimer;

    private DcMotor intake = null;

    private boolean hasIntake = false;

    private Servo arm = null;


    private enum PATHSTATE {
        PRELOAD_TO_SCORE,
        SHOOT,
        SHOOT_UPDATE,

        SCORE_TO_COLLECT,

        COLLECT_TO_END,

        RETURN_TO_SCORE,

        SHOOT_COLLECT,

        SHOOT_COLLECT_UPDATE,
        SCORE_TO_LEAVE,
        END,

        EXIT
    };

    private enum SHOOTSTATE {

        SHOOT_1,
       SHOOT_1_UPDATE,

        INTAKE,

        SHOOT_2,
        SHOOT_2_UPDATE,
        SHOOT_END
    }

    SHOOTSTATE shootstate;

    PATHSTATE pathState;

    private final Pose scorePose = new Pose(78.353, 16.706, Math.toRadians(240)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose collectPose = new Pose(100.706, 34.353, Math.toRadians(0));
    private final Pose collectendPose = new Pose(129.176, 34.118, Math.toRadians((0)));
    private final Pose endPose = new Pose(99.765, 16.235, Math.toRadians(240));


    private Path launchtoCollectline;

    private PathChain collectline;

    private PathChain returnline, leavelaunchline;

    FeederLauncher leftFeederLauncher = new FeederLauncher();

    FeederLauncher rightFeederLauncher = new FeederLauncher();

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        launchtoCollectline = new Path(new BezierLine(scorePose, collectPose));
        launchtoCollectline.setLinearHeadingInterpolation(scorePose.getHeading(), collectPose.getHeading());

        collectline = follower.pathBuilder()
                .addPath(new BezierLine(collectPose, collectendPose))
                .setLinearHeadingInterpolation(collectPose.getHeading(), collectendPose.getHeading())
                .build();



        /* This is our leavelaunchline PathChain. We are using a single path with a BezierLine, which is a straight line. */
        returnline = follower.pathBuilder()
                .addPath(new BezierLine(collectendPose, scorePose))
                .setLinearHeadingInterpolation(collectendPose.getHeading(), scorePose.getHeading())
                .build();

        leavelaunchline = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();


    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case PRELOAD_TO_SCORE:
            case SHOOT:
                /* Shoot */
                actionTimer.reset();
                setPathState(PATHSTATE.SHOOT_UPDATE);


                break;
            case SHOOT_UPDATE:
                shootUpdate();

                break;

            case SCORE_TO_COLLECT:

                follower.followPath(launchtoCollectline);
                setPathState(PATHSTATE.COLLECT_TO_END);
                break;

            case COLLECT_TO_END:
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    intake.setPower(-1.0);
                    follower.followPath(collectline);
                    setPathState(PATHSTATE.RETURN_TO_SCORE);
                }
                break;
            case RETURN_TO_SCORE:
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    intake.setPower(0);
                    follower.followPath(returnline);
                    setPathState(PATHSTATE.SHOOT_COLLECT);
                }
                break;
            case SHOOT_COLLECT:
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    actionTimer.reset();
                    shootstate = SHOOTSTATE.SHOOT_1;
                    setPathState(PATHSTATE.SHOOT_COLLECT_UPDATE);
                }
                break;
            case SHOOT_COLLECT_UPDATE:
                shootCollectUpdate();
                break;
            case SCORE_TO_LEAVE:
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    follower.followPath(leavelaunchline);
                    setPathState(PATHSTATE.END);
                }
                break;

            case END:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(PATHSTATE.EXIT);
                }
                break;
        }

    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(PATHSTATE state) {
        pathState = state;
        pathTimer.reset();
    }

    public void shootArtifacts() {
        leftFeederLauncher.launch(3150,0,true, true);
        rightFeederLauncher.launch(3150,0,true,true);
    }


    public void shootUpdate() {
        switch (shootstate) {
            case SHOOT_1:
                shootArtifacts();
                shootstate = SHOOTSTATE.SHOOT_1_UPDATE;

                break;


            case SHOOT_1_UPDATE:
                if (rightFeederLauncher.getStatus() == FeederLauncher.LaunchState.LAUNCHED) {
                    shootstate = SHOOTSTATE.INTAKE;
                }
                break;

            case INTAKE:
                if (hasIntake) {
                    intake.setPower(-1.0);
                    intakeTimer.reset();
                }
                shootstate = SHOOTSTATE.SHOOT_2;
                break;

            case SHOOT_2:

                if (intakeTimer.milliseconds() >= 600) {
                    intake.setPower(0.0);
                    shootArtifacts();
                    shootstate = SHOOTSTATE.SHOOT_2_UPDATE;
                }

                break;

            case SHOOT_2_UPDATE:
                if (rightFeederLauncher.getStatus() == FeederLauncher.LaunchState.LAUNCHED) {
                    shootstate = SHOOTSTATE.SHOOT_END;
                }
                break;

            case SHOOT_END:
                setPathState(PATHSTATE.SCORE_TO_COLLECT);
                break;
        }

        //launch left and right. Start right just before releasing the left feeder
        leftFeederLauncher.update();
        rightFeederLauncher.update();

        //recovery based on timeout
        if (actionTimer.milliseconds() >= 13000) {
            setPathState(PATHSTATE.SCORE_TO_COLLECT);
        }

        telemetry.addData("Shoot State: ", shootstate);

    }

    public void shootCollectUpdate() {
        switch (shootstate) {
            case SHOOT_1:
                shootArtifacts();
                shootstate = SHOOTSTATE.SHOOT_1_UPDATE;

                break;


            case SHOOT_1_UPDATE:
                if (rightFeederLauncher.getStatus() == FeederLauncher.LaunchState.LAUNCHED) {
                    shootstate = SHOOTSTATE.INTAKE;
                }
                break;

            case INTAKE:
                if (hasIntake) {
                    intake.setPower(-1.0);
                    intakeTimer.reset();
                }
                shootstate = SHOOTSTATE.SHOOT_2;
                break;

            case SHOOT_2:

                if (intakeTimer.milliseconds() >= 600) {
                    intake.setPower(0.0);
                    shootArtifacts();
                    shootstate = SHOOTSTATE.SHOOT_2_UPDATE;
                }

                break;

            case SHOOT_2_UPDATE:
                if (rightFeederLauncher.getStatus() == FeederLauncher.LaunchState.LAUNCHED) {
                    shootstate = SHOOTSTATE.SHOOT_END;
                }
                break;

            case SHOOT_END:
                setPathState(PATHSTATE.SCORE_TO_LEAVE);
                break;
        }

        //launch left and right. Start right just before releasing the left feeder
        leftFeederLauncher.update();
        rightFeederLauncher.update();

        //recovery based on timeout
        if (actionTimer.milliseconds() >= 13000) {
            setPathState(PATHSTATE.SCORE_TO_LEAVE);
        }

        telemetry.addData("Shoot State: ", shootstate);

    }



    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("LeftFeederLauncher state",leftFeederLauncher.getStatus());
        telemetry.addData("RightFeederLauncher state",rightFeederLauncher.getStatus());

        leftFeederLauncher.reportHardwareStatus();
        rightFeederLauncher.reportHardwareStatus();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
        actionTimer = new ElapsedTime();
        intakeTimer = new ElapsedTime();
        opmodeTimer.reset();

        //Shootstate

        shootstate = SHOOTSTATE.SHOOT_1;

        //leftfeederlauncher
        leftFeederLauncher.init(hardwareMap,telemetry,"launcher","feederServoLeft");

        rightFeederLauncher.init(hardwareMap,telemetry,"launcher","feederServoRight");

        //arm servo



        intake = hardwareMap.get(DcMotor.class, "intake");

        arm = hardwareMap.get(Servo.class, "armServo");

        hasIntake = (intake != null);

        if (hasIntake) {
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(scorePose);

        telemetry.addData("intake", hasIntake     ? "OK" : "MISSING");

        arm.scaleRange(0.2, 0.8);
        //arm.setPosition();

    }



    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.reset();
        setPathState(PATHSTATE.PRELOAD_TO_SCORE);

        //start the flywheel early
        leftFeederLauncher.launch(0,0,false, false);
        rightFeederLauncher.launch(0,0,false, false);
        //intake.setPower(-1.0);

        arm.setPosition(0.5);

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        leftFeederLauncher.stop();
        rightFeederLauncher.stop();
    }
}










