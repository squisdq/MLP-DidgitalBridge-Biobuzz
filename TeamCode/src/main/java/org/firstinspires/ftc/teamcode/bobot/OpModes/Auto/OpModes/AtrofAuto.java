package org.firstinspires.ftc.teamcode.bobot.OpModes.Auto.OpModes;

import static com.pedropathing.api.Paths.*;

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bobot.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.Controllers.MasterController;
import org.firstinspires.ftc.teamcode.bobot.Utils.MLPConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Disabled
@Autonomous(name = "AtrofAuto", group = "!0Auto")
public class AtrofAuto extends LinearOpMode {

    private Follower follower;

    private MasterController masterController;

    private final double sT = 3000, iT = 1000;
    private final Pose start                = new Pose(56, 8, Math.PI);
    private final Pose intakeRowStart       = new Pose(56, 8, Math.PI);
    private final Pose intakeRow            = new Pose(6.803, 8.356, Math.PI);
    private final Pose toIntakeFlower1      = new Pose(48.2864, 115.5877, Math.PI / 2);
    private final Pose intakeFlower1        = new Pose(47.7202, 130.0364, Math.PI / 2);
    private final Pose toShootFlower1       = new Pose(48.495, 116.3858, 3 * Math.PI / 2);
    private final Pose kurva                = new Pose(52.0695, 11.9288, 3.03136);
    private final Pose kurvaControl1        = new Pose(70.7334, 108.9735, 0);
    private final Pose kurvaControl2        = new Pose(70.9843, 67.7923, 0);
    private final Pose kurvaControl3        = new Pose(110.8104, 4.539, 0);
    private final Pose toIntakeFlower2Start = new Pose(52.0695, 11.9288, Math.PI);
    private final Pose toIntakeFlower2      = new Pose(22.0199, 47.3146, Math.PI);
    private final Pose intakeFlower2        = new Pose(10.5083, 47.3974, Math.PI);
    private final Pose shootFlower2         = new Pose(56.7964, 25.2583, Math.PI);
    private final Pose park                 = new Pose(9.8593, 97.351, 0);

    // Autonomous routine

    private Command intakeOn()  { return instant(masterController::intake); }
    private Command intakeOff() { return instant(masterController::stopIntake); }
    private Command outtake()   { return instant(masterController::outtake); }

    private Command shooterOn(boolean on) { return instant(() -> masterController.setShooter(on)); }

    private Command shootCommand(double ms) {
        return sequential(
                race(
                        infinite(masterController::shoot),   // вызывается каждый цикл
                        waitMs(ms)                 // время стрельбы
                ),
                instant(masterController::stopIntake)    // сброс флагов, блокиратор закрыт, интейк 0
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
                waitMs(200),
                waitUntil(masterController::atTarget),
                shootCommand(sT),
                intakeOn(),
                follow(follower, intakeRow()),
                intakeOff(),
                follow(follower, toIntakeFlower1()),
                intakeOn(),
                setArm(IntakeController.armStates.flower),
                follow(follower, intakeFlower1()),
                waitMs(iT),
                intakeOff(),
                setArm(IntakeController.armStates.up),
                follow(follower, toShootFlower1()),
                shootCommand(sT),
                intakeOn(),
                follow(follower, kurva()),
                intakeOff(),
                shootCommand(sT),
                follow(follower, toIntakeFlower2()),
                intakeOn(),
                follow(follower, intakeFlower2()),
                waitMs(iT),
                intakeOff(),
                setArm(IntakeController.armStates.up),
                follow(follower, shootFlower2()),
                shootCommand(sT),
                follow(follower, park()),
                stopOpMode()
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        masterController = new MasterController(hardwareMap, follower); // сначала master
        follower.setPose(start);                                        // потом поза
        follower.update();
        masterController.setShooter(true);

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }
            masterController.update();
            telemetry.update();
        }

        Pose p = follower.pose();
        MLPConfig.startPose = new Pose(p.x(), p.y(), p.heading());
    }

    public Path intakeRow() {
        return line(intakeRowStart, intakeRow).constant(intakeRow);
    }

    public Path toIntakeFlower1() {
        return line(intakeRow, toIntakeFlower1).linear(intakeRow, toIntakeFlower1);
    }

    public Path intakeFlower1() {
        return line(toIntakeFlower1, intakeFlower1).linear(toIntakeFlower1, intakeFlower1);
    }

    public Path toShootFlower1() {
        return line(intakeFlower1, toShootFlower1).linear(intakeFlower1, toShootFlower1);
    }

    public Path kurva() {
        return curve(toShootFlower1, kurvaControl1, kurvaControl2, kurvaControl3, kurva).tangent();
    }

    public Path toIntakeFlower2() {
        return line(toIntakeFlower2Start, toIntakeFlower2).linear(toIntakeFlower2Start, toIntakeFlower2);
    }

    public Path intakeFlower2() {
        return line(toIntakeFlower2, intakeFlower2).linear(toIntakeFlower2, intakeFlower2);
    }

    public Path shootFlower2() {
        return line(intakeFlower2, shootFlower2).linear(intakeFlower2, shootFlower2);
    }

    public Path park() {
        return line(shootFlower2, park).linear(shootFlower2, park);
    }
}