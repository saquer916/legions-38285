package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.paths.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;

// Start in the start corner, shoot the preload at the raised CELL (turret aims itself), then drive
// forward out of the corner. Hands pose and alliance to TeleOp.
@Autonomous(name = "Shoot Preload", group = "Competition", preselectTeleOp = "Main TeleOp")
public class ShootPreloadAuto extends OpMode {
    // The yaw servo needs ~4 s for a 130 deg swing; feeding earlier throws the ball off target (simulation).
    public static double SPIN_UP_TIMEOUT = 5.0;
    // Feeding ends once no ball has gone through for EMPTY_TIME, or after FEED_TIME at the latest.
    public static double EMPTY_TIME = 2.0;
    public static double FEED_TIME = 6.0;
    public static double LEAVE_DISTANCE = 24;
    public static double LEAVE_TIMEOUT = 3.0;
    public static double LEAVE_TOLERANCE = 2.0;

    private enum Step { SPIN_UP, FEED, LEAVE, DONE }

    private Robot robot;
    private Step step;
    private Pose leavePose;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void init_loop() {
        // X = blue, B = red, A flips which CELL starts raised.
        if (gamepad1.x) robot.shooter.setAlliance(Alliance.BLUE);
        if (gamepad1.b) robot.shooter.setAlliance(Alliance.RED);
        if (gamepad1.a) robot.shooter.setFarCellUp(true);
        if (gamepad1.y) robot.shooter.setFarCellUp(false);
        telemetry.addData("Alliance (X blue / B red)", robot.shooter.getAlliance());
        telemetry.addData("Raised CELL (A far / Y audience)", robot.shooter.isFarCellUp() ? "far" : "audience");
        telemetry.addData("Place robot at", FieldConstants.startPose(robot.shooter.getAlliance()));
        if (!robot.gate.isConnected()) telemetry.addData("Gate servo", "NOT IN CONFIG");
    }

    @Override
    public void start() {
        Alliance alliance = robot.shooter.getAlliance();
        PoseStorage.alliance = alliance;

        Pose start = FieldConstants.startPose(alliance);
        robot.follower.setPose(start);
        robot.follower.hold(start);
        leavePose = new Pose(
                start.x() + LEAVE_DISTANCE * Math.cos(start.heading()),
                start.y() + LEAVE_DISTANCE * Math.sin(start.heading()),
                start.heading());

        robot.shooter.setSpinning(true);
        step = Step.SPIN_UP;
        timer.reset();
    }

    @Override
    public void loop() {
        switch (step) {
            case SPIN_UP:
                if (robot.shooter.isReady() || timer.seconds() > SPIN_UP_TIMEOUT) {
                    robot.feeder.feed();
                    step = Step.FEED;
                    timer.reset();
                }
                break;
            case FEED:
                robot.feeder.feed();
                if (robot.feeder.secondsSinceShot() > EMPTY_TIME || timer.seconds() > FEED_TIME) {
                    robot.feeder.stop();
                    robot.shooter.setSpinning(false);
                    robot.follower.hold(leavePose);
                    step = Step.LEAVE;
                    timer.reset();
                }
                break;
            case LEAVE:
                if (robot.follower.pose().distance(leavePose) < LEAVE_TOLERANCE || timer.seconds() > LEAVE_TIMEOUT) {
                    step = Step.DONE;
                }
                break;
            default:
                break;
        }

        robot.update();

        telemetry.addData("Step", step);
        telemetry.addData("Pose", robot.follower.pose());
        telemetry.addData("Flywheel rpm", "%.0f / %.0f", robot.shooter.flywheel.getRpm(), robot.shooter.flywheel.getTargetRpm());
    }
}
