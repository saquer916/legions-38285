package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

// Pedro field coordinates, inches. Viewed from the audience: origin at the near corner on the red
// alliance wall, +x toward the blue wall, +y away from the audience, heading 0 = +x, CCW positive.
// Same orientation as Figure 9-17 in the BIOBUZZ manual (section 9). Auto start poses must use this frame.
public final class FieldConstants {
    public static double FIELD_SIZE = 144;

    // 9.6: the HIVE structure sits in the centre of the field, red HIVE on the red side.
    // Figure 9-10: HIVE centre to centre 25.5 in.
    public static double HIVE_SPACING = 25.5;

    // Horizontal distance from a HIVE's pivot to the opening centre of its raised CELL, along the
    // audience axis. Estimated from Figures 9-9/9-10 (CELLS 18.84 in apart, 12.04 in deep, tipped 30 deg).
    // TODO: check against the official field CAD. The raised opening is 53.5 to 65.6 in above the tiles.
    public static double RAISED_CELL_OFFSET = 8.0;

    public static Pose hivePivot(Alliance alliance) {
        double side = alliance == Alliance.RED ? -1 : 1;
        return new Pose(FIELD_SIZE / 2 + side * HIVE_SPACING / 2, FIELD_SIZE / 2);
    }

    // Each HIVE tips so one CELL is up. farCellUp = the CELL away from the audience is the raised one
    // (AprilTags 30-33 red, 42-45 blue); otherwise the audience CELL (34-37 red, 38-41 blue).
    public static Pose raisedCell(Alliance alliance, boolean farCellUp) {
        Pose pivot = hivePivot(alliance);
        return pivot.withY(pivot.y() + (farCellUp ? 1 : -1) * RAISED_CELL_OFFSET);
    }

    private FieldConstants() {
    }
}
