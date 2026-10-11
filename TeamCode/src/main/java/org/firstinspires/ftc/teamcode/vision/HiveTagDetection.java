package org.firstinspires.ftc.teamcode.vision;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import android.util.Size;

public class HiveTagDetection implements Subsystem {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Alliance alliance;
    private AprilTagClusterDetection target;

    public HiveTagDetection(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        aprilTag = new AprilTagProcessor.Builder()
            .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .build();


        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, RobotConstants.webcam))
            .addProcessor(aprilTag)
            .setCameraResolution(new Size(640, 480))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .setAutoStopLiveView(true)
            .build();      
    }

    @Override 
    public void update() {
        target = null;

        List<AprilTagDetection> curDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : curDetections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;

                if (clusterDet.ftcPose != null && clusterDet.metadata.name.startsWith(alliance.name())) {
                    target = clusterDet;
                }

            }
        }
    }

    public boolean hasTarget() {
        return target != null;
    }
    public double getRange() {
        return (!hasTarget()) ? null : target.ftcPose.range;
    }
    public double getBearing() {
        return (!hasTarget()) ? null : target.ftcPose.bearing;
    }
    public void close() {
        visionPortal.close();
    }


}
