package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

// Feeds the shooter one ball at a time and owns the gate. Each ball pulls the flywheel ~10% down, so a ball right behind
// it leaves slow and falls short (simulation). After every dip the stack stops until the flywheel is back at speed and
// the turret is on target, for at most MAX_HOLD_TIME. Call feed() every loop while shooting; a loop without it ends the
// burst. Registered right after the Shooter so it sees this loop's flywheel speed and sets intake, transfer and gate
// before they update.
public class Feeder implements Subsystem {
    // A drop this far below the wheel's speed before the ball means a ball just went through.
    public static double SHOT_DIP_RPM = 120;
    // Back within this of the pre-shot speed (or at the target) counts as recovered.
    public static double RECOVERED_RPM = 75;
    // Flywheel must stay recovered this long before the next ball goes.
    public static double SETTLE_TIME = 0.05;
    // Never hold the next ball longer than this: a flywheel that can't quite reach its target still shoots.
    public static double MAX_HOLD_TIME = 1.0;
    // Pushing this long since the last release with nothing through: it's the last ball (nothing behind it to
    // double-feed), and as built it has to coast up the final climb on its own, so push at full power.
    public static double STALL_TIME = 0.6;
    // After a burst, back the stack off this long before closing the gate so the blade doesn't close on a ball.
    public static double GATE_CLEAR_TIME = 0.15;

    private final Robot robot;
    private final ElapsedTime settleTimer = new ElapsedTime();
    private final ElapsedTime holdTimer = new ElapsedTime();
    private final ElapsedTime sinceRelease = new ElapsedTime();
    private final ElapsedTime sinceShot = new ElapsedTime();
    private final ElapsedTime clearTimer = new ElapsedTime();
    private boolean requested = false;
    private boolean active = false;
    private boolean waiting = false;
    private boolean clearing = false;
    // Flywheel speed before the ball now in flight: follows the wheel up, and down with the target.
    private double referenceRpm = 0;

    Feeder(Robot robot) {
        this.robot = robot;
    }

    // Call every loop while shooting.
    public void feed() {
        requested = true;
    }

    // End the burst now (also cancels a feed() earlier in this loop).
    public void stop() {
        requested = false;
        robot.intake.setState(Intake.State.OFF);
        robot.transfer.setState(Transfer.State.OFF);
    }

    @Override
    public void update() {
        if (requested) {
            if (!active) startBurst();
            active = true;
            clearing = false;
            holdOrPush();
        } else if (active) {
            active = false;
            clearing = true;
            clearTimer.reset();
        }
        if (clearing) {
            if (clearTimer.seconds() < GATE_CLEAR_TIME) {
                robot.intake.setState(Intake.State.OFF);
                robot.transfer.setState(Transfer.State.REVERSE);
            } else {
                clearing = false;
            }
        }
        robot.gate.setOpen(active || clearing);
        requested = false;
    }

    private void startBurst() {
        waiting = false;
        referenceRpm = robot.shooter.flywheel.getRpm();
        sinceRelease.reset();
        sinceShot.reset();
    }

    private void holdOrPush() {
        Flywheel flywheel = robot.shooter.flywheel;
        double rpm = flywheel.getRpm();
        if (!waiting) {
            referenceRpm = Math.min(Math.max(referenceRpm, rpm), flywheel.getTargetRpm() + Flywheel.READY_TOLERANCE_RPM);
            if (referenceRpm - rpm > SHOT_DIP_RPM) {
                waiting = true;
                holdTimer.reset();
                settleTimer.reset();
                sinceShot.reset();
            }
        } else {
            boolean recovered = (flywheel.atSpeed() || referenceRpm - rpm < RECOVERED_RPM) && robot.shooter.isAimed();
            if (!recovered) settleTimer.reset();
            if ((recovered && settleTimer.seconds() > SETTLE_TIME) || holdTimer.seconds() > MAX_HOLD_TIME) {
                waiting = false;
                referenceRpm = rpm;
                sinceRelease.reset();
            }
        }

        // Metered speed: slow enough that the next ball can still be stopped short of the wheel.
        Transfer.State push = sinceRelease.seconds() > STALL_TIME ? Transfer.State.FEED : Transfer.State.METER;
        robot.transfer.setState(waiting ? Transfer.State.OFF : push);
        robot.intake.setState(waiting ? Intake.State.OFF : Intake.State.IN);
    }

    public boolean isWaiting() {
        return waiting;
    }

    // Seconds since the last ball went through the flywheel (or since the burst started).
    public double secondsSinceShot() {
        return sinceShot.seconds();
    }
}
