package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.paths.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {
    private Robot robot;
    private final Gamepad previous = new Gamepad();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.follower.setPose(PoseStorage.currentPose);
    }

    @Override
    public void init_loop() {
        // X = blue, B = red. Defaults to whatever auto ran.
        if (gamepad1.x) robot.shooter.setAlliance(Alliance.BLUE);
        if (gamepad1.b) robot.shooter.setAlliance(Alliance.RED);
        telemetry.addData("Alliance (X blue / B red)", robot.shooter.getAlliance());
        telemetry.addData("Start pose", PoseStorage.currentPose);
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

        Shooter shooter = robot.shooter;
        if (gamepad1.y && !previous.y) shooter.setSpinning(!shooter.isSpinning());
        if (gamepad1.a && !previous.a) shooter.setFarCellUp(!shooter.isFarCellUp());
        if (gamepad1.dpad_left && !previous.dpad_left) shooter.trimAim(1);
        if (gamepad1.dpad_right && !previous.dpad_right) shooter.trimAim(-1);
        if (gamepad1.dpad_up && !previous.dpad_up) shooter.trimRpm(1);
        if (gamepad1.dpad_down && !previous.dpad_down) shooter.trimRpm(-1);
        if (gamepad1.back && !previous.back) shooter.resetTrim();
        previous.copy(gamepad1);

        robot.update();

        telemetry.addData("Pose", robot.follower.pose());
        telemetry.addData("Shooter", shooter.isReady() ? "READY" : shooter.isReachable() ? "aiming" : "TURN ROBOT");
        telemetry.addData("Raised CELL (A to flip)", shooter.isFarCellUp() ? "far" : "audience");
        telemetry.addData("CELL distance (in)", "%.1f", shooter.getDistance());
        telemetry.addData("Flywheel rpm", "%.0f / %.0f", shooter.flywheel.getRpm(), shooter.flywheel.getTargetRpm());
        telemetry.addData("Turret deg", "%.1f / %.1f", Math.toDegrees(shooter.turret.getAngle()), Math.toDegrees(shooter.turret.getTarget()));
        telemetry.addData("Trim", "%.0f deg, %.0f rpm", Math.toDegrees(shooter.getAimTrim()), shooter.getRpmTrim());
    }
}
