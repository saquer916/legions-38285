package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.paths.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;

// gamepad1:
//   sticks drive, RB slow mode
//   LT intake, LB reverse intake + transfer
//   RT shoot (spins up, feeds once aimed and at speed), Y flywheel on/off
//   A flip which CELL is raised, dpad trims aim (L/R) and rpm (U/D)
//   back: robot squared into the start corner -> reset pose and trims
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends OpMode {
    private Robot robot;
    private final Gamepad previous = new Gamepad();
    private boolean poseFromAuto;
    private boolean feeding = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        Pose stored = PoseStorage.currentPose;
        poseFromAuto = stored.x() != 0 || stored.y() != 0;
        robot.follower.setPose(stored);
    }

    @Override
    public void init_loop() {
        // X = blue, B = red. Defaults to whatever auto ran.
        if (gamepad1.x) robot.shooter.setAlliance(Alliance.BLUE);
        if (gamepad1.b) robot.shooter.setAlliance(Alliance.RED);
        telemetry.addData("Alliance (X blue / B red)", robot.shooter.getAlliance());
        if (!robot.gate.isConnected()) telemetry.addData("Gate servo", "NOT IN CONFIG");
        telemetry.addData("Start pose", poseFromAuto ? PoseStorage.currentPose + " (from auto)"
                : FieldConstants.startPose(robot.shooter.getAlliance()) + " (place robot in the start corner)");
    }

    @Override
    public void start() {
        PoseStorage.alliance = robot.shooter.getAlliance();
        if (!poseFromAuto) robot.follower.setPose(FieldConstants.startPose(robot.shooter.getAlliance()));
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
        if (gamepad1.back && !previous.back) {
            robot.follower.setPose(FieldConstants.startPose(shooter.getAlliance()));
            shooter.resetTrim();
        }

        boolean shooting = gamepad1.right_trigger > 0.5;
        boolean intaking = gamepad1.left_trigger > 0.5;
        if (shooting) shooter.setSpinning(true);
        // Once a burst starts keep it going; the Feeder holds each next ball until the flywheel recovers and the turret
        // is back on target.
        feeding = shooting && (feeding || shooter.isReady());

        if (gamepad1.left_bumper) {
            robot.intake.setState(Intake.State.REVERSE);
            robot.transfer.setState(Transfer.State.REVERSE);
        } else if (feeding) {
            robot.feeder.feed();
        } else if (intaking) {
            robot.intake.setState(Intake.State.IN);
            robot.transfer.setState(Transfer.State.STAGE);
        } else {
            robot.intake.setState(Intake.State.OFF);
            robot.transfer.setState(Transfer.State.OFF);
        }
        previous.copy(gamepad1);

        robot.update();

        telemetry.addData("Pose", robot.follower.pose());
        telemetry.addData("Shooter", shooter.isReady() ? "READY" : !shooter.isReachable() ? "TURN ROBOT"
                : !shooter.isInRange() ? "TOO FAR" : "aiming");
        if (feeding) telemetry.addData("Feeder", robot.feeder.isWaiting() ? "holding for flywheel / aim" : "feeding");
        telemetry.addData("Raised CELL (A to flip)", shooter.isFarCellUp() ? "far" : "audience");
        telemetry.addData("CELL distance (in)", "%.1f", shooter.getDistance());
        telemetry.addData("Flywheel rpm", "%.0f / %.0f", shooter.flywheel.getRpm(), shooter.flywheel.getTargetRpm());
        telemetry.addData("Turret deg", "%.1f / %.1f", Math.toDegrees(shooter.turret.getAngle()), Math.toDegrees(shooter.turret.getTarget()));
        telemetry.addData("Trim", "%.0f deg, %.0f rpm", Math.toDegrees(shooter.getAimTrim()), shooter.getRpmTrim());
    }

    @Override
    public void stop() {
        // Only an auto that just ran should hand its pose to the next TeleOp.
        PoseStorage.currentPose = Pose.zero();
    }
}
