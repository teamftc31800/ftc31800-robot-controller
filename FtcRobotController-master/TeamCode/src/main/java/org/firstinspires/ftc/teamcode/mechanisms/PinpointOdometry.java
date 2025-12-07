package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;


public class PinpointOdometry {

    private final GoBildaPinpointDriver pinpoint;
    private Pose2D currentPose;

    public PinpointOdometry(
            HardwareMap hardwareMap,
            double startX,
            double startY,
            double startHeadingDeg,
            double xOffset,
            double yOffset,
            GoBildaPinpointDriver.EncoderDirection reverseXEncoder,
            GoBildaPinpointDriver.EncoderDirection reverseYEncoder
    ) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure encoder direction and mounting offsets
        pinpoint.setEncoderDirections(reverseXEncoder, reverseYEncoder);
        pinpoint.setOffsets(xOffset, yOffset,DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // Set the starting pose of the robot
        currentPose = new Pose2D(DistanceUnit.INCH, startX, startY, AngleUnit.DEGREES, startHeadingDeg);
        pinpoint.setPosition(currentPose);
    }

    /** Must be called every loop */
    public void update() {
        pinpoint.update();
        currentPose = pinpoint.getPosition();
    }

    public Pose2D getPose() {
        return currentPose;
    }

    public double getX() { return currentPose.getX(DistanceUnit.INCH); }
    public double getY() { return currentPose.getY(DistanceUnit.INCH); }
    public double getHeadingDeg() { return currentPose.getHeading(AngleUnit.DEGREES); }

    public void setPose(double x, double y, double headingDeg) {
        currentPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, headingDeg);
        pinpoint.setPosition(currentPose);
    }

    public double getVelocityX() { return pinpoint.getVelX(DistanceUnit.INCH); }
    public double getVelocityY() { return pinpoint.getVelY(DistanceUnit.INCH); }
    public double getAngularVelocityDeg() { return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES); }
}
