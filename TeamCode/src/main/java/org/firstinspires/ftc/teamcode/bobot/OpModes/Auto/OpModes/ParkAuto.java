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
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bobot.Utils.MLPConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "ParkAuto", group = "!0Auto")
public class ParkAuto extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(7.7401, 134.9752, 90);
    private final Pose path1Start = poseFactory.of(7.7401, 134.9752, 90);
    private final Pose path1 = poseFactory.of(8.2086, 117.0579, 90);
    private Command stopOpMode(){
        return instant(this::requestOpModeStop);
    }

    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                stopOpMode()
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

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

            telemetry.update();
            Pose p = follower.pose();
            MLPConfig.startPose = new Pose(p.x(), p.y(), p.heading());
        }
    }

    public Path path1() {
        return line(path1Start, path1).linear(path1Start, path1);
    }
}