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

    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("FLMotor");
                c.backLeftName.set("BLMotor");
                c.frontRightName.set("FRMotor");
                c.backRightName.set("BRMotor");

                c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);

                c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-6.236851008858268);
        c.yPodOffset.set(-1.0752279176486759);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2891172043025035);
                Controller secondaryTranslationalForward = Controller.proportional(0.10682108163579622);
                Controller primaryTranslationalLateral = Controller.proportional(0.4768758474663588);
                Controller secondaryTranslationalLateral = Controller.proportional(0.17619288328149593);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.008202454317615683));
                c.brake.set(Controller.proportionalFeedforward(0.00697208616997333));

                c.headingFeedback.set(Controller.proportional(3.8203467256160644));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.07464662932092812, 0.0052579063154966806));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10995241351732359, 0.34778203650712747));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002263537188492559, -0.0018327329240921232));

                c.maxAchievableForwardVelocity.set(98.18985287403356);
                c.maxAchievableStrafeVelocity.set(78.25847773088618);
                c.naturalForwardDeceleration.set(25.90833909277645);
                c.naturalStrafeDeceleration.set(61.08213878179955);
            }
    );


    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}