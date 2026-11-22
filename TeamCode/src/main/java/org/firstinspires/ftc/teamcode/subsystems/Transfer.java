package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;

// Surgical-tubing kicker rows K3 -> K2 -> K1 up the ramp, one goBILDA 5203 3.7:1 through GT2 belts.
public class Transfer implements Subsystem {
    // STAGE carries balls up to the shooter without forcing them into a stopped flywheel; the tubing slips.
    public static double STAGE_POWER = 0.5;
    public static double FEED_POWER = 1.0;
    public static double REVERSE_POWER = -0.6;
    public static boolean REVERSED = false;

    public enum State { OFF, STAGE, FEED, REVERSE }

    private final DcMotorEx motor;
    private State state = State.OFF;

    public Transfer(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotConstants.transfer);
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Brake so the stack doesn't roll back down the ramp.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void update() {
        switch (state) {
            case STAGE:
                motor.setPower(STAGE_POWER);
                break;
            case FEED:
                motor.setPower(FEED_POWER);
                break;
            case REVERSE:
                motor.setPower(REVERSE_POWER);
                break;
            default:
                motor.setPower(0);
        }
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }
}
