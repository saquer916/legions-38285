package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;

// Two REV HD Hex motors (UltraPlanetary) on one flywheel shaft. Velocity loop on the left
// motor's encoder, same power to both so they never fight each other.
public class Flywheel implements Subsystem {
    public static double MOTOR_TICKS_PER_REV = 28;
    // Motor revolutions per flywheel revolution. TODO: set to the UltraPlanetary cartridges on the robot.
    public static double GEAR_RATIO = 1.0;

    // power = (kS + kV * rpm) * 12V / battery + kP * error
    public static double kS = 0.0;
    public static double kV = 1.0 / 6000;
    public static double kP = 0.0005;
    public static double NOMINAL_VOLTAGE = 12.0;
    public static double READY_TOLERANCE_RPM = 75;

    private final DcMotorEx left;
    private final DcMotorEx right;
    private final VoltageSensor battery;
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private double voltage;
    private double targetRpm = 0;
    private double currentRpm = 0;

    public Flywheel(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, RobotConstants.flywheelLeft);
        right = hardwareMap.get(DcMotorEx.class, RobotConstants.flywheelRight);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx motor : new DcMotorEx[]{left, right}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        battery = hardwareMap.voltageSensor.iterator().next();
        voltage = battery.getVoltage();
    }

    @Override
    public void update() {
        // Voltage isn't in the bulk read, so only poll it twice a second.
        if (voltageTimer.seconds() > 0.5) {
            voltage = battery.getVoltage();
            voltageTimer.reset();
        }

        currentRpm = left.getVelocity() * 60.0 / (MOTOR_TICKS_PER_REV * GEAR_RATIO);

        double power = 0;
        if (targetRpm > 0) {
            power = (kS + kV * targetRpm) * NOMINAL_VOLTAGE / voltage + kP * (targetRpm - currentRpm);
        }
        // Never drive backwards; let it coast down.
        power = Range.clip(power, 0, 1);

        left.setPower(power);
        right.setPower(power);
    }

    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0, rpm);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getRpm() {
        return currentRpm;
    }

    public boolean atSpeed() {
        return targetRpm > 0 && Math.abs(targetRpm - currentRpm) < READY_TOLERANCE_RPM;
    }
}
