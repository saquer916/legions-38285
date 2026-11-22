package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.paths.PoseStorage;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.InterpLUT;

// Aims the yaw turret at the raised CELL of the alliance HIVE and sets flywheel speed from distance, all from odometry.
// The hood is fixed, so flywheel RPM is the only range adjustment.
public class Shooter implements Subsystem {
    // Turret axis relative to Pedro's robot centre, robot frame, inches (+x toward the intake, +y left).
    public static double TURRET_OFFSET_X = 0.1;
    public static double TURRET_OFFSET_Y = 0.0;
    // Direction the shooter fires at turret angle 0, relative to robot forward. PI = out the back.
    public static double TURRET_ZERO_HEADING = Math.PI;

    // Turret axis to raised CELL distance (in) -> flywheel RPM. TODO: starting guesses, tune on the field.
    public static InterpLUT RPM_BY_DISTANCE = new InterpLUT()
            .add(24, 2400)
            .add(48, 2800)
            .add(72, 3200)
            .add(96, 3600)
            .add(120, 4000)
            .add(144, 4400);

    // Distance (in) -> ball flight time (s), used to lead the shot while driving. TODO: measure.
    public static InterpLUT FLIGHT_TIME_BY_DISTANCE = new InterpLUT()
            .add(24, 0.35)
            .add(144, 0.9);
    public static boolean LEAD_WHILE_MOVING = false;

    public static double AIM_TRIM_STEP = Math.toRadians(1);
    public static double RPM_TRIM_STEP = 50;

    public final Turret turret;
    public final Flywheel flywheel;
    private final Follower follower;

    private Alliance alliance = PoseStorage.alliance;
    private boolean spinning = false;
    private boolean farCellUp = true;
    private double aimTrim = 0;
    private double rpmTrim = 0;
    private double distance = 0;
    private boolean reachable = false;

    public Shooter(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        turret = new Turret(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
    }

    @Override
    public void update() {
        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        double heading = pose.heading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double turretX = pose.x() + TURRET_OFFSET_X * cos - TURRET_OFFSET_Y * sin;
        double turretY = pose.y() + TURRET_OFFSET_X * sin + TURRET_OFFSET_Y * cos;

        Pose cell = FieldConstants.raisedCell(alliance, farCellUp);
        double dx = cell.x() - turretX;
        double dy = cell.y() - turretY;
        distance = Math.hypot(dx, dy);

        if (LEAD_WHILE_MOVING) {
            // Aim where the CELL will be relative to us when the ball gets there.
            double flightTime = FLIGHT_TIME_BY_DISTANCE.get(distance);
            dx -= velocity.vx * flightTime;
            dy -= velocity.vy * flightTime;
            distance = Math.hypot(dx, dy);
        }

        double desired = AngleUnit.normalizeRadians(Math.atan2(dy, dx) - heading - TURRET_ZERO_HEADING + aimTrim);
        reachable = desired >= Turret.MIN_ANGLE && desired <= Turret.MAX_ANGLE;
        turret.setTarget(desired);
        // Counter-rotate against the robot's own turning.
        turret.setFeedforwardRate(-velocity.omega);

        flywheel.setTargetRpm(spinning ? RPM_BY_DISTANCE.get(distance) + rpmTrim : 0);

        turret.update();
        flywheel.update();
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    // The HIVE tips when its raised CELL fills, so the driver flips this to match the field.
    public void setFarCellUp(boolean farCellUp) {
        this.farCellUp = farCellUp;
    }

    public boolean isFarCellUp() {
        return farCellUp;
    }

    public void setSpinning(boolean spinning) {
        this.spinning = spinning;
    }

    public boolean isSpinning() {
        return spinning;
    }

    // direction: +1 / -1. Trims nudge the odometry aim when it has drifted.
    public void trimAim(int direction) {
        aimTrim += direction * AIM_TRIM_STEP;
    }

    public void trimRpm(int direction) {
        rpmTrim += direction * RPM_TRIM_STEP;
    }

    public void resetTrim() {
        aimTrim = 0;
        rpmTrim = 0;
    }

    public double getAimTrim() {
        return aimTrim;
    }

    public double getRpmTrim() {
        return rpmTrim;
    }

    public double getDistance() {
        return distance;
    }

    // Whether the turret could face the raised CELL from this pose (no lead), without waiting for update().
    public boolean canReach(Pose pose) {
        Pose cell = FieldConstants.raisedCell(alliance, farCellUp);
        double desired = AngleUnit.normalizeRadians(
                Math.atan2(cell.y() - pose.y(), cell.x() - pose.x()) - pose.heading() - TURRET_ZERO_HEADING + aimTrim);
        return desired >= Turret.MIN_ANGLE && desired <= Turret.MAX_ANGLE;
    }

    // Robot heading that puts the raised CELL straight down the turret's zero.
    public double facingHeading(Pose pose) {
        Pose cell = FieldConstants.raisedCell(alliance, farCellUp);
        return Math.atan2(cell.y() - pose.y(), cell.x() - pose.x()) - TURRET_ZERO_HEADING;
    }

    // False when the CELL is behind the turret's soft limits; turn the robot.
    public boolean isReachable() {
        return reachable;
    }

    public boolean isReady() {
        return spinning && reachable && flywheel.atSpeed() && turret.onTarget();
    }
}
