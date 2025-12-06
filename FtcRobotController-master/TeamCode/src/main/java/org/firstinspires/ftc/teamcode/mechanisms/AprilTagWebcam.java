package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        AprilTagLibrary.Builder tagbuilder = new AprilTagLibrary.Builder();
        tagbuilder.addTag(20, "Tag20", 0.166, DistanceUnit.METER);
        tagbuilder.addTag(21, "Tag21", 0.166, DistanceUnit.METER);
        tagbuilder.addTag(22, "Tag22", 0.166, DistanceUnit.METER);
        tagbuilder.addTag(23, "Tag23", 0.166, DistanceUnit.METER);
        tagbuilder.addTag(24, "Tag24", 0.166, DistanceUnit.METER);

// ...add other tags

        AprilTagLibrary library = tagbuilder.build();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagLibrary(library)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }



    public void displayDetectionTelemetry(AprilTagDetection detectionId) {
        if (detectionId != null) {
            if (detectionId.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detectionId.id, detectionId.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectionId.ftcPose.x, detectionId.ftcPose.y, detectionId.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectionId.ftcPose.pitch, detectionId.ftcPose.roll, detectionId.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectionId.ftcPose.range, detectionId.ftcPose.bearing, detectionId.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectionId.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectionId.center.x, detectionId.center.y));
            }
        }
    }



    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }


    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    public void reportHardwareStatus() {
        telemetry.addLine("Webcam 1");
    }


}
