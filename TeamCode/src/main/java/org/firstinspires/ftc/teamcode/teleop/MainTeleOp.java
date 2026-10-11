package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.paths.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.follower.setPose(PoseStorage.currentPose);
    }

    @Override
    public void loop() {
        double speed;
        if (gamepad1.right_bumper) {
            speed = 0.4;
        }
        else speed = 1.0;
        robot.follower.manual(
            -gamepad1.left_stick_y * speed,
            -gamepad1.left_stick_x * speed,
            -gamepad1.right_stick_x * speed
        );

        robot.update();

        telemetry.addData("Pose", robot.follower.pose());
    }
}
