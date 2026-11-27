package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.paths.PoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    public final Follower follower;
    public final Shooter shooter;
    public final Intake intake;
    public final Transfer transfer;
    public final Gate gate;
    public final Feeder feeder;

    private final List<LynxModule> hubs;
    private final List<Subsystem> subsystems = new ArrayList<>();

    public Robot(HardwareMap hardwareMap) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.create(hardwareMap);
        shooter = register(new Shooter(hardwareMap, follower));
        // After the shooter (fresh flywheel speed), before intake, transfer and gate (it sets their states).
        feeder = register(new Feeder(this));
        intake = register(new Intake(hardwareMap));
        transfer = register(new Transfer(hardwareMap));
        gate = register(new Gate(hardwareMap));
    }

    private <T extends Subsystem> T register(T subsystem) {
        subsystems.add(subsystem);
        return subsystem;
    }

    public void update() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        follower.update();
        PoseStorage.currentPose = follower.pose();

        for (Subsystem subsystem : subsystems) {
            subsystem.update();
        }
    }
}
