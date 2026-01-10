package org.firstinspires.ftc.teamcode.pedroPathing;  // adjust to your package

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Callback Example Auto", group = "Examples")
@Disabled
public class CallbackExampleAuto extends OpMode {
    private Follower follower;
    private PathChain driveAndDoAction;
    private final Pose startPose = new Pose(10, 10, Math.toRadians(90));
    private final Pose targetPose = new Pose(50, 50, Math.toRadians(90));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);  // assuming you have a Constants helper
        follower.setStartingPose(startPose);

        // Build a path chain with a straight BezierLine
        driveAndDoAction = follower.pathBuilder()
                .addPath(new BezierLine(startPose, targetPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())

                // Add a parametric callback: when 50% of the path is traveled, do action
                .addParametricCallback(0.5, () -> {
                    // Example action: extend arm, start intake, or whatever
                    telemetry.addData("Callback", "Halfway! Running action...");
                    telemetry.update();

                    // e.g., robot.arm.setPosition(ARM_EXTENDED);
                    //       robot.intake.run(…);
                })

                // Optionally: add a pose callback as well
                .addPoseCallback(
                        new Pose(48, 48, 90),            // a pose along the path to trigger at
                        () -> {
                            telemetry.addData("Callback", "Passed near (40,40)!");
                            telemetry.update();
                        },
                        0.5  // initial guess for parametric t
                )

                .build();
    }

    @Override
    public void start() {
        follower.followPath(driveAndDoAction);
    }

    @Override
    public void loop() {
        follower.update();

        // optionally check isBusy or other conditions
        if (!follower.isBusy()) {
            telemetry.addData("Status", "Done driving");
            telemetry.update();
            // maybe end auto or do more
        }
    }

    @Override
    public void stop() {
        // nothing special
    }
}
