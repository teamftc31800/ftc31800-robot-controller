
package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.mechanisms.FeederLauncher;

@Autonomous(name = "RedTeamBotAuto", group = "Examples")
public class RedTeamBotAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private enum PATHSTATE {
        PRELOAD_TO_SCORE,
        SHOOT,
        SHOOT_UPDATE,
        SCORE_TO_LEAVE,
        END,

        EXIT
    };

    PATHSTATE pathState;

    private final Pose startPose = new Pose(125.78, 124.07, Math.toRadians(225)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(103.30, 102.45, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose endPose = new Pose(110.3529411764706, 95.76470588235296, Math.toRadians(45)); // Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose pickup2Pose = new Pose(24, 24, Math.toRadians(90)); // Middle (Second Set) of Artifacts from the Spike Mark.
//    private final Pose pickup3Pose = new Pose(12, 12, Math.toRadians(90)); // Lowest (Third Set) of Artifacts from the Spike Mark.

 //   private final Pose startPose = new Pose(0, 0, Math.toRadians(90)); // Start Pose of our robot.
 //   private final Pose scorePose = new Pose(0, 48, Math.toRadians(270)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
//

    private Path scorePreload;
    private PathChain leaveLaunchline;

    FeederLauncher leftFeederLauncher = new FeederLauncher();

    FeederLauncher rightFeederLauncher = new FeederLauncher();

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our leavelaunchline PathChain. We are using a single path with a BezierLine, which is a straight line. */
        leaveLaunchline = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
               .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
               .build();

    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case PRELOAD_TO_SCORE:
                follower.followPath(scorePreload);
                setPathState(PATHSTATE.SHOOT);
                break;
            case SHOOT:
                if(!follower.isBusy()) {
                    /* Shoot */
                    actionTimer.resetTimer();
                    shootArtifacts();
                    setPathState(PATHSTATE.SHOOT_UPDATE);
                }
                break;

            case SHOOT_UPDATE:
                shootUpdate();
                break;
            case SCORE_TO_LEAVE:


           /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
           - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                    if (!follower.isBusy()) {
                        /* Score Preload */

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(leaveLaunchline, true);
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
        pathTimer.resetTimer();
    }

    public void shootArtifacts() {
        leftFeederLauncher.launch(0,0,true, true);
        rightFeederLauncher.launch(0,0,true,true);
    }


    public void shootUpdate() {

        //launch left and right. Start right just before releasing the left feeder
        leftFeederLauncher.update();
        if (leftFeederLauncher.getStatus() == FeederLauncher.LaunchState.LAUNCHING) {

            rightFeederLauncher.update();
        }

        //when left and right has shooted, ready to leave.
        if ((leftFeederLauncher.getStatus() == FeederLauncher.LaunchState.LAUNCHED)
        && (rightFeederLauncher.getStatus() == FeederLauncher.LaunchState.LAUNCHED)) {
            setPathState(PATHSTATE.SCORE_TO_LEAVE);
        }

        //recovery based on timeout
        if (actionTimer.getElapsedTimeSeconds() > 10) {
            setPathState(PATHSTATE.SCORE_TO_LEAVE);
        }
    }



    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

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
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();

        //leftfeederlauncher
        leftFeederLauncher.init(hardwareMap,telemetry,"launcher","feederServoLeft");

        rightFeederLauncher.init(hardwareMap,telemetry,"null","feederServoRight");

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(PATHSTATE.PRELOAD_TO_SCORE);

        //start the flywheel early
        leftFeederLauncher.launch(0,0,false, false);
        rightFeederLauncher.launch(0,0,false, false);

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        leftFeederLauncher.stop();
        rightFeederLauncher.stop();
    }
}

