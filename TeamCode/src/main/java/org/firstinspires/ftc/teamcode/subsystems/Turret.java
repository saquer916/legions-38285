package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;

// Yaw turret: goBILDA 2000-0025-0002 in continuous mode driving a 24T pinion on the 214T ring.
// Angle comes from an external encoder read through a motor port's encoder input.
// Angles are radians, CCW positive seen from above, 0 = turret position at init.
public class Turret implements Subsystem {
    public static double RING_TEETH = 214;
    // TODO: encoder isn't in the CAD yet. Defaults assume a REV Through Bore on its own 24T pinion on the ring.
    public static double ENCODER_PINION_TEETH = 24;
    public static double ENCODER_TICKS_PER_REV = 8192;
    public static boolean REVERSE_ENCODER = false;
    public static boolean REVERSE_SERVO = false;

    // Soft limits so the wiring can't wind up.
    public static double MIN_ANGLE = Math.toRadians(-170);
    public static double MAX_ANGLE = Math.toRadians(170);

    // power = kP * error + kD * dError/dt + kV * feedforward rate + kS * sign(error)
    public static double kP = 2.0;
    public static double kD = 0.05;
    public static double kV = 0.3;
    public static double kS = 0.04;
    public static double ON_TARGET_TOLERANCE = Math.toRadians(2);

    private final CRServo servo;
    private final DcMotorEx encoder;
    private final int zeroTicks;
    private final ElapsedTime timer = new ElapsedTime();

    private double angle = 0;
    private double target = 0;
    private double feedforwardRate = 0;
    private double lastError = 0;
    private boolean enabled = true;

    public Turret(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, RobotConstants.turretServo);
        servo.setDirection(REVERSE_SERVO ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // The port may also run a motor, so never reset it; just remember where we started.
        encoder = hardwareMap.get(DcMotorEx.class, RobotConstants.turretEncoder);
        zeroTicks = encoder.getCurrentPosition();
    }

    @Override
    public void update() {
        double ticksPerTurretRev = ENCODER_TICKS_PER_REV * RING_TEETH / ENCODER_PINION_TEETH;
        angle = (encoder.getCurrentPosition() - zeroTicks) * (REVERSE_ENCODER ? -1 : 1)
                * 2 * Math.PI / ticksPerTurretRev;

        double dt = timer.seconds();
        timer.reset();
        double error = target - angle;
        double derivative = dt > 0 ? (error - lastError) / dt : 0;
        lastError = error;

        double power = 0;
        if (enabled) {
            power = kP * error + kD * derivative + kV * feedforwardRate;
            if (Math.abs(error) > ON_TARGET_TOLERANCE) power += Math.signum(error) * kS;
        }
        servo.setPower(Range.clip(power, -1, 1));
    }

    // Clamped to the soft limits; if the goal is outside them the driver has to turn the robot.
    public void setTarget(double radians) {
        target = Range.clip(radians, MIN_ANGLE, MAX_ANGLE);
    }

    // Expected turret rate (rad/s) to hold the aim, e.g. cancelling the robot's own rotation.
    public void setFeedforwardRate(double radiansPerSecond) {
        feedforwardRate = radiansPerSecond;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public double getAngle() {
        return angle;
    }

    public double getTarget() {
        return target;
    }

    public boolean onTarget() {
        return Math.abs(target - angle) < ON_TARGET_TOLERANCE;
    }
}
