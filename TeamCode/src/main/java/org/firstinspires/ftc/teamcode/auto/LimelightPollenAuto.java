package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.paths.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.vision.BallDetection;

// Shoots the preload, then loops: Limelight finds the nearest POLLEN, odometry turns that into a field
// position, the robot drives over it with the intake running, and after a few pickups it turns to the
// raised CELL and shoots. Stops everything near the end of auto.
@Autonomous(name = "Limelight Pollen", group = "Competition", preselectTeleOp = "Main TeleOp")
public class LimelightPollenAuto extends OpMode {
    // Limelight lens relative to the robot centre in the robot frame (in): +forward toward the intake, +left.
    // Lens of the Limelight on its mount (CAD: A 166, W 70.5, Z 218.5 mm; robot centre A -60.5).
    public static double CAMERA_FORWARD = 8.9;
    public static double CAMERA_LEFT = 2.8;
    // Robot centre to the intake roller, and how far past the ball the roller should go.
    public static double INTAKE_REACH = 8.4;
    public static double OVERDRIVE = 3.0;

    // Which detections are worth chasing.
    public static double MAX_BALL_DISTANCE = 60;
    public static double WALL_MARGIN = 8;
    // Keep-out box around the HIVE frame (49.46 x 38.95 in base, 9.6.1) plus half a robot.
    public static double HIVE_KEEP_OUT_X = 36;
    public static double HIVE_KEEP_OUT_Y = 31;
    public static boolean OWN_HALF_ONLY = true;
    // Keep re-aiming at the ball until this close; after that it is below the camera's view.
    public static double REFINE_DISTANCE = 18;
    public static double SAME_BALL_RADIUS = 8;

    public static double LEAVE_DISTANCE = 24;
    public static double SEARCH_STEP = Math.toRadians(35);
    public static int SEARCH_STEPS = 10;
    public static double SEARCH_DWELL = 0.6;
    public static double DRIVE_TIMEOUT = 3.5;
    public static double ARRIVE_TOLERANCE = 1.5;
    public static int BALLS_PER_TRIP = 3;

    // The yaw servo needs ~4 s for a 130 deg swing; feeding earlier throws the ball off target (simulation).
    public static double SPIN_UP_TIMEOUT = 5.0;
    // Feeding ends once no ball has gone through for EMPTY_TIME, or after FEED_TIME at the latest.
    public static double EMPTY_TIME = 2.0;
    public static double FEED_TIME = 6.0;
    public static double LAST_PICKUP_TIME = 21;
    public static double STOP_TIME = 28;

    private enum Step { AIM, FEED, LEAVE, SEARCH, DRIVE_TO_BALL, DONE }

    private Robot robot;
    private BallDetection balls;
    private Step step;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();

    private Pose ball;
    private Pose approach;
    private Pose leaveTarget;
    private int searchSteps;
    private int pickups;
    private boolean leftStart = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        balls = new BallDetection(hardwareMap, robot.shooter.getAlliance());
    }

    @Override
    public void init_loop() {
        // X = blue, B = red, A / Y = far / audience CELL starts raised.
        if (gamepad1.x) robot.shooter.setAlliance(Alliance.BLUE);
        if (gamepad1.b) robot.shooter.setAlliance(Alliance.RED);
        if (gamepad1.a) robot.shooter.setFarCellUp(true);
        if (gamepad1.y) robot.shooter.setFarCellUp(false);
        telemetry.addData("Alliance (X blue / B red)", robot.shooter.getAlliance());
        telemetry.addData("Raised CELL (A far / Y audience)", robot.shooter.isFarCellUp() ? "far" : "audience");
        telemetry.addData("Place robot at", FieldConstants.startPose(robot.shooter.getAlliance()));
        telemetry.addData("Limelight", balls.isCameraConnected() ? "connected" : "NOT CONNECTED");
        if (!robot.gate.isConnected()) telemetry.addData("Gate servo", "NOT IN CONFIG");
    }

    @Override
    public void start() {
        PoseStorage.alliance = robot.shooter.getAlliance();
        Pose start = FieldConstants.startPose(robot.shooter.getAlliance());
        robot.follower.setPose(start);
        matchTimer.reset();
        startAim();
    }

    @Override
    public void loop() {
        balls.update();
        Pose pose = robot.follower.pose();

        if (step != Step.DONE && matchTimer.seconds() > STOP_TIME) {
            stopMechanisms();
            robot.follower.hold(pose);
            step = Step.DONE;
        }

        switch (step) {
            case AIM:
                if (robot.shooter.isReady() || timer.seconds() > SPIN_UP_TIMEOUT) {
                    robot.feeder.feed();
                    next(Step.FEED);
                }
                break;
            case FEED:
                robot.feeder.feed();
                if (robot.feeder.secondsSinceShot() > EMPTY_TIME || timer.seconds() > FEED_TIME) {
                    stopMechanisms();
                    if (matchTimer.seconds() > LAST_PICKUP_TIME) {
                        robot.follower.hold(pose);
                        next(Step.DONE);
                    } else if (!leftStart) {
                        // Rotating in the start corner would hit the walls, so drive out first.
                        leftStart = true;
                        leaveTarget = ahead(pose, LEAVE_DISTANCE);
                        robot.follower.hold(leaveTarget);
                        next(Step.LEAVE);
                    } else {
                        startSearch(pose);
                    }
                }
                break;
            case LEAVE:
                if (pose.distance(leaveTarget) < ARRIVE_TOLERANCE || timer.seconds() > DRIVE_TIMEOUT) {
                    startSearch(pose);
                }
                break;
            case SEARCH:
                Pose found = nearestBall(pose);
                if (found != null) {
                    driveTo(found, pose);
                    robot.intake.setState(Intake.State.IN);
                    robot.transfer.setState(Transfer.State.STAGE);
                    next(Step.DRIVE_TO_BALL);
                } else if (matchTimer.seconds() > LAST_PICKUP_TIME) {
                    finishPickups();
                } else if (timer.seconds() > SEARCH_DWELL) {
                    if (++searchSteps > SEARCH_STEPS) {
                        finishPickups();
                    } else {
                        robot.follower.hold(pose.withHeading(pose.heading() + SEARCH_STEP));
                        timer.reset();
                    }
                }
                break;
            case DRIVE_TO_BALL:
                if (pose.distance(ball) > REFINE_DISTANCE) {
                    Pose seen = nearestBall(pose);
                    if (seen != null && seen.distance(ball) < SAME_BALL_RADIUS) driveTo(seen, pose);
                }
                if (pose.distance(approach) < ARRIVE_TOLERANCE || timer.seconds() > DRIVE_TIMEOUT) {
                    pickups++;
                    if (pickups % BALLS_PER_TRIP == 0 || matchTimer.seconds() > LAST_PICKUP_TIME) {
                        finishPickups();
                    } else {
                        startSearch(pose);
                    }
                }
                break;
            default:
                break;
        }

        robot.update();

        telemetry.addData("Step", step);
        telemetry.addData("Time", "%.1f", matchTimer.seconds());
        telemetry.addData("Pickups", pickups);
        telemetry.addData("Pose", pose);
        telemetry.addData("Ball target", ball);
        telemetry.addData("Balls seen", balls.getDetections().size());
    }

    @Override
    public void stop() {
        balls.close();
    }

    private void next(Step step) {
        this.step = step;
        timer.reset();
    }

    private void startAim() {
        Pose pose = robot.follower.pose();
        robot.shooter.setSpinning(true);
        // The turret covers +-170 deg; only turn the robot if the CELL is outside that.
        robot.follower.hold(robot.shooter.canReach(pose) ? pose : pose.withHeading(robot.shooter.facingHeading(pose)));
        next(Step.AIM);
    }

    private void startSearch(Pose pose) {
        searchSteps = 0;
        robot.follower.hold(pose);
        next(Step.SEARCH);
    }

    // Done collecting for this trip: turn to the CELL if needed and shoot.
    private void finishPickups() {
        robot.intake.setState(Intake.State.OFF);
        robot.transfer.setState(Transfer.State.OFF);
        startAim();
    }

    private void stopMechanisms() {
        robot.feeder.stop();
        robot.shooter.setSpinning(false);
    }

    // Drive so the roller passes OVERDRIVE inches beyond the ball, facing it.
    private void driveTo(Pose target, Pose pose) {
        ball = target;
        double heading = Math.atan2(target.y() - pose.y(), target.x() - pose.x());
        double back = INTAKE_REACH - OVERDRIVE;
        approach = new Pose(target.x() - back * Math.cos(heading), target.y() - back * Math.sin(heading), heading);
        robot.follower.hold(approach);
    }

    private Pose ahead(Pose pose, double distance) {
        return new Pose(pose.x() + distance * Math.cos(pose.heading()), pose.y() + distance * Math.sin(pose.heading()), pose.heading());
    }

    // Nearest POLLEN the camera sees, as a field position, or null if none is worth chasing.
    private Pose nearestBall(Pose pose) {
        Pose best = null;
        double bestDistance = Double.MAX_VALUE;
        double cos = Math.cos(pose.heading());
        double sin = Math.sin(pose.heading());

        for (LLResultTypes.DetectorResult detection : balls.getDetections()) {
            if (!detection.getClassName().equals("yellow_pollen")) continue;
            double forward = balls.getForwardDistance(detection);
            if (Double.isNaN(forward) || forward <= 0 || forward > MAX_BALL_DISTANCE) continue;
            double left = balls.getLeftDistance(detection);

            double rx = CAMERA_FORWARD + forward;
            double ry = CAMERA_LEFT + left;
            Pose field = new Pose(pose.x() + rx * cos - ry * sin, pose.y() + rx * sin + ry * cos);
            if (!reachable(field)) continue;

            double distance = Math.hypot(rx, ry);
            if (distance < bestDistance) {
                bestDistance = distance;
                best = field;
            }
        }
        return best;
    }

    private boolean reachable(Pose ball) {
        double size = FieldConstants.FIELD_SIZE;
        double centre = size / 2;
        if (ball.x() < WALL_MARGIN || ball.x() > size - WALL_MARGIN) return false;
        if (ball.y() < WALL_MARGIN || ball.y() > size - WALL_MARGIN) return false;
        if (Math.abs(ball.x() - centre) < HIVE_KEEP_OUT_X && Math.abs(ball.y() - centre) < HIVE_KEEP_OUT_Y) return false;
        if (OWN_HALF_ONLY) {
            boolean red = robot.shooter.getAlliance() == Alliance.RED;
            if (red ? ball.x() > centre : ball.x() < centre) return false;
        }
        return true;
    }
}
