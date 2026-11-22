package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;

// SWYFT roller, goBILDA 5203 3.7:1 belted 1:1.
public class Intake implements Subsystem {
    public static double IN_POWER = 1.0;
    public static double REVERSE_POWER = -0.6;
    public static boolean REVERSED = false;

    public enum State { OFF, IN, REVERSE }

    private final DcMotorEx motor;
    private State state = State.OFF;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotConstants.intake);
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void update() {
        switch (state) {
            case IN:
                motor.setPower(IN_POWER);
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
