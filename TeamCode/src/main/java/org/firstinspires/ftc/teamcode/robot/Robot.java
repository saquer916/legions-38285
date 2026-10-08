package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    private final List<LynxModule> hubs;
    private final List<Subsystem> subsystems = new ArrayList<>();

    public Robot(HardwareMap hardwareMap) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private <T extends Subsystem> T register(T subsystem) {
        subsystems.add(subsystem);
        return subsystem;
    }

    public void update() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        for (Subsystem subsystem : subsystems) {
            subsystem.update();
        }
    }
}
