package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;

// Servo gate at the top of the ramp's vertical chute, just under the roof (CAD: 'od ball gate').
// A goBILDA 2000-0025-0002 swings a printed blade across the chute: closed holds the ball stack,
// open is 57 deg CCW seen from above (0.19 of the servo's 300 deg range). Check the direction on the robot.
// The Feeder opens and closes it. Optional in the hardware config so a robot without it still drives.
public class Gate implements Subsystem {
    public static double CLOSED_POSITION = 0.15;
    public static double OPEN_POSITION = 0.34;

    private final Servo servo;
    private boolean open = false;

    public Gate(HardwareMap hardwareMap) {
        servo = hardwareMap.tryGet(Servo.class, RobotConstants.gate);
        // Closed from init on, holding the preloaded stack.
        if (servo != null) servo.setPosition(CLOSED_POSITION);
    }

    @Override
    public void update() {
        if (servo != null) servo.setPosition(open ? OPEN_POSITION : CLOSED_POSITION);
    }

    public void setOpen(boolean open) {
        this.open = open;
    }

    // False when the config has no "gate" servo; show it in init telemetry.
    public boolean isConnected() {
        return servo != null;
    }
}
