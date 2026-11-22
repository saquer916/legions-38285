package org.firstinspires.ftc.teamcode.vision;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BallDetection implements Subsystem {

    private static final int MAX_STALENESS_MS = 120;
    private static final double MIN_CONFIDENCE = 0.5;
    private static final double CAMERA_HEIGHT = 9.0; // inches
    private static final double CAMERA_TILT = 21.0; // degrees
    private static final double POLLEN_CENTER_HEIGHT = 1.4; // inches
    private static final double NECTAR_CENTER_HEIGHT = 1.8; // inches


    private Limelight3A limelight;
    private Alliance alliance;
    private ArrayList<LLResultTypes.DetectorResult> results = new ArrayList<LLResultTypes.DetectorResult>();

    public BallDetection(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;

        limelight = hardwareMap.tryGet(Limelight3A.class, RobotConstants.limelight);
        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0);
            limelight.start();
            
        }
    }

    public boolean isCameraConnected() {
        if (limelight != null) return limelight.isConnected();
        else return false;
    }
    public void close() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    @Override 
    public void update() {
        results.clear();
        if (limelight == null) {
            return;
        }
        LLResult latestResult = limelight.getLatestResult();
        if (latestResult.getStaleness() > MAX_STALENESS_MS) return;
        List<LLResultTypes.DetectorResult> latest = latestResult.getDetectorResults();
        for (LLResultTypes.DetectorResult detection : latest) {
            double confidence = detection.getConfidence();
            if (confidence > 1) confidence /= 100;
            if (confidence < MIN_CONFIDENCE) continue;
            if (isOpponentNectar(detection.getClassName())) {
                continue;
            }
            results.add(detection);
        }
    }

    public boolean isOpponentNectar(String className) {
        if (alliance == Alliance.BLUE) {
            return className.equals("red_nectar");
        }
        else {
            return className.equals("blue_nectar");
        }
    }

    public double getForwardDistance(LLResultTypes.DetectorResult detection) {
        double ty = detection.getTargetYDegrees();
        double angleBelowHorizontal = CAMERA_TILT - ty;
        if (angleBelowHorizontal <= 5) {
            return Double.NaN;
        }
        double ballCenterHeight = 0;
        if (detection.getClassName().equals("yellow_pollen")) {
            ballCenterHeight = POLLEN_CENTER_HEIGHT;
        }
        else {
            ballCenterHeight = NECTAR_CENTER_HEIGHT;
        }

        double heightDifference = CAMERA_HEIGHT - ballCenterHeight;
        double forward = heightDifference / Math.tan(Math.toRadians(angleBelowHorizontal));
        
        return forward;
    }

    public double getLeftDistance(LLResultTypes.DetectorResult detection) {
        double forward = getForwardDistance(detection);
        double tx = detection.getTargetXDegrees();
        double left = - (forward * Math.tan(Math.toRadians(tx)));
        return left;
    }

    public ArrayList<LLResultTypes.DetectorResult> getDetections() {
        return results;
    }
}
