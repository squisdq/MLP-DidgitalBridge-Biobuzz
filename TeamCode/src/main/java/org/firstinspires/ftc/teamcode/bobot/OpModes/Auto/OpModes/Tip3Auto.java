package org.firstinspires.ftc.teamcode.bobot.OpModes.Auto.OpModes;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.bobot.Controllers.IntakeController.armStates.flower;
import static org.firstinspires.ftc.teamcode.bobot.Controllers.IntakeController.armStates.up;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bobot.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.Controllers.MasterController;
import org.firstinspires.ftc.teamcode.bobot.Utils.MLPConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "TipAuto", group = "!0Auto")
public class Tip3Auto extends LinearOpMode {

    private Follower follower;
    private MasterController masterController;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54.1196, 6.5897, 90);
    private final Pose intakeCurveFlower1 = poseFactory.of(93.8891, 12.047, 270);
    private final Pose intakeCurveFlower1Control1 = poseFactory.of(52.4752, 39.0199, 0);
    private final Pose intakeCurveFlower1Control2 = poseFactory.of(92.1192, 36.6639, 0);
    private final Pose transit = poseFactory.of(52.2169, 110.6937, 180);
    private final Pose transitControl1 = poseFactory.of(57.0762, 64.6788, 0);
    private final Pose transitControl2 = poseFactory.of(91, 62, 0);
    private final Pose toShootFlower1 = poseFactory.of(60.0886, 117.6792, 0);
    private final Pose intakeFlower2 = poseFactory.of(46.245, 126.303, 90);
    private final Pose shootFlower2 = poseFactory.of(58.6225, 113.9934, 0);
    private final Pose toIntakeFlower3 = poseFactory.of(23.9412, 44.4259, 180);
    private final Pose toIntakeFlower3Control1 = poseFactory.of(15.1871, 92.4272, 0);
    private final Pose toIntakeFlower3Control2 = poseFactory.of(58.3493, 47.5315, 0);
    private final Pose intakeFlower3 = poseFactory.of(14.851, 47.755, -177.066);
    private final Pose toShootFlower3 = poseFactory.of(35.1258, 23.5513, 131.3152);
    private final Pose intakeRow = poseFactory.of(9.0828, 5.4603, 172.8511);
    private final Pose intakeRowControl1 = poseFactory.of(32.4189, 2.4255, 0);
    private final Pose toPark = poseFactory.of(10.4321, 95.9652, 90);
//    private final Pose toParkControl1 = poseFactory.of(30.8444, 68, 0);

    private final double sT = 1700, iT = 1400;
    private Command intakeOn()  { return instant(masterController::intake); }
    private Command intakeOff() { return instant(masterController::stopIntake); }
    private Command outtake()   { return instant(masterController::outtake); }

    private Command shooterOn(boolean on) { return instant(() -> masterController.setShooter(on)); }

    private Command shootCommand(double ms) {
        return sequential(
                shooterOn(true),
                waitUntil(masterController::atTarget),
                race(
                        infinite(masterController::shoot),
                        waitMs(ms)
                ),
                instant(masterController::stopIntake),
                shooterOn(false)
        );
    }

    private Command setArm(IntakeController.armStates state) {
        return instant(() -> masterController.setArmState(state));
    }

    private Command stopOpMode(){
        return instant(this::requestOpModeStop);
    }

    public Command autoRoutine() {
        return sequential(
                setArm(flower),
                waitUntil(masterController::atTarget),
                shootCommand(sT),
                intakeOn(),
                follow(follower, intakeCurveFlower1()),
                waitMs(iT),
                intakeOff(),
                follow(follower, transit()),
                setArm(up),
                follow(follower, toShootFlower1()),
                setArm(flower),
                shootCommand(sT),
                intakeOn(),
                follow(follower, intakeFlower2()),
                waitMs(iT),
                intakeOff(),
                setArm(up),
                follow(follower, shootFlower2()),
                shootCommand(sT),
                setArm(flower),
                follow(follower, toIntakeFlower3()),
                intakeOn(),
                follow(follower, intakeFlower3()),
                waitMs(iT),
                intakeOff(),
                follow(follower, toShootFlower3()),
                setArm(up),
                shootCommand(sT),
                follow(follower, toPark()),
                stopOpMode()
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        masterController = new MasterController(hardwareMap, follower);
        masterController.setShooter(true);
        masterController.setSPower(0.4);
        masterController.initServo();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            masterController.update();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
        Pose p = follower.pose();
        MLPConfig.startPose = new Pose(p.x(), p.y(), p.heading());
    }

    public Path intakeCurveFlower1() {
        return curve(start, intakeCurveFlower1Control1, intakeCurveFlower1Control2, intakeCurveFlower1).tangent();
    }

    public Path transit() {
        return line(intakeCurveFlower1, transit).reverseTangent();
    }

    public Path toShootFlower1() {
        return line(transit, toShootFlower1).constant(270);
    }

    public Path intakeFlower2() {
        return line(toShootFlower1, intakeFlower2).constant(intakeFlower2);
    }

    public Path shootFlower2() {
        return line(intakeFlower2, toShootFlower1).linear(shootFlower2, toShootFlower1);
    }

    public Path toIntakeFlower3() {
        return curve(toShootFlower1, toIntakeFlower3Control1, toIntakeFlower3Control2, toIntakeFlower3).tangent();
    }

    public Path intakeFlower3() {
        return line(toIntakeFlower3, intakeFlower3).constant(toIntakeFlower3);
    }

    public Path toShootFlower3() {
        return curve(toIntakeFlower3, toIntakeFlower3Control2, toIntakeFlower3Control1, toShootFlower1).reverseTangent();
    }

    public Path intakeRow() {
        return curve(toShootFlower3, intakeRowControl1, intakeRow).tangent();
    }

    public Path toPark() {
        return line(toShootFlower1, toPark).constant(toShootFlower1);
    }
}