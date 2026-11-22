package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.Alliance;

public final class PoseStorage {
    public static Pose currentPose = Pose.zero();
    // Set by auto so teleop aims at the right hive
    public static Alliance alliance = Alliance.BLUE;

    private PoseStorage() {
    }
}
