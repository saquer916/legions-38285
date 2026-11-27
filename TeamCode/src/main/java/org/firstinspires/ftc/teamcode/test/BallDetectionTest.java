package org.firstinspires.ftc.teamcode.test;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.vision.BallDetection;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Ball Detection Test", group="Test")
public class BallDetectionTest extends OpMode {
    
    private BallDetection ballDetection;

    @Override
    public void init() {
        ballDetection = new BallDetection(hardwareMap, Alliance.RED);
    }

    @Override
    public void loop() {
        ballDetection.update();
        telemetry.addData("Camera connection status", ballDetection.isCameraConnected());

        ArrayList<LLResultTypes.DetectorResult> detections = ballDetection.getDetections();
        telemetry.addData("Ball count", detections.size());

        for (LLResultTypes.DetectorResult detection : detections) {
            telemetry.addData("name", detection.getClassName());
            telemetry.addData("tx", detection.getTargetXDegrees());
            telemetry.addData("ty", detection.getTargetYDegrees());
            telemetry.addData("confidence score", detection.getConfidence());
            telemetry.addData("forward distance", ballDetection.getForwardDistance(detection));
            telemetry.addData("horizontal distance", ballDetection.getLeftDistance(detection));
        }
        telemetry.update();
    }

    @Override 
    public void stop() {
        ballDetection.close();
    }

    
}
