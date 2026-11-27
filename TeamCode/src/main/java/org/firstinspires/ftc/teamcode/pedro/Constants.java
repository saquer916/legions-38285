package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        // From the CAD: the forward (X) pod rides on the robot's centre line (rear face of the front cross channel), the strafe
        // (Y) pod sits 89.9 mm behind the drivetrain's centre (front face of the rear cross channel). Directions still need
        // the Pinpoint tuner on the real robot.
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(-3.54);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // Starting values from the simulator for the CAD robot: 11.6 kg incl. battery (PLA prints, 6 mm plywood, goBILDA
    // parts), yaw inertia 0.30 kg m^2, 96 mm mecanum on 5203 312 rpm motors. Strafe is set to 82 % of forward for the
    // mecanum roller losses the sim doesn't model. Run ForesightTuner on the real robot and replace these.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.maxAchievableForwardVelocity.set(60.5);           // in/s
        c.maxAchievableStrafeVelocity.set(49.6);
        c.naturalForwardDeceleration.set(26.5);             // in/s^2, coasting at zero power
        c.naturalStrafeDeceleration.set(26.5);
        c.linearBrakeCoefficients.set(Matrix.diag(0.00083, 0.00083));    // braking distance = a v + b v^2
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.00322, 0.00393));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0265, 0.00881));
        c.forwardTranslational.set(Controller.piecewise(Controller.proportional(0.03)).put(2.5, Controller.proportional(0.15)));
        c.strafeTranslational.set(Controller.piecewise(Controller.proportional(0.04)).put(2.5, Controller.proportional(0.18)));
        c.coast.set(Controller.proportionalFeedforward(1.0 / 60.5));
        c.brake.set(Controller.proportionalFeedforward(1.0 / 60.5));
        c.headingFeedback.set(Controller.proportional(1.2));
    });

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
