package org.firstinspires.ftc.teamcode.bobot.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bobot.Controllers.MasterController;
import org.firstinspires.ftc.teamcode.bobot.Utils.MLPConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@TeleOp(group = "!0TeleOp", name = "SoloPOP")
public class SoloOp extends OpMode {
    MasterController masterController;
    Follower follower;
    MultipleTelemetry multipleTelemetry;

    public static double speed = 2100, angle = 0.72;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(MLPConfig.startPose);
        follower.update();

        masterController = new MasterController(hardwareMap, follower);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    }

    @Override
    public void loop() {
        if(gamepad1.left_trigger >= 0.2)
            masterController.intake();
        else if(gamepad1.left_bumper)
            masterController.outtake();
        else if(gamepad1.right_trigger >= 0.2)
            masterController.shoot();
        else
            masterController.stopIntake();
        if(gamepad1.rightBumperWasPressed())
            masterController.toggleShooter();

        if(gamepad1.startWasPressed())
            masterController.resetPositionRight();
        if(gamepad1.backWasPressed())
            masterController.resetPositionLeft();
        if(gamepad1.xWasPressed())
            masterController.sweepTMode();
        if(gamepad1.bWasPressed())
            masterController.sweepGoal();
        if(gamepad1.yWasPressed())
            masterController.switchArmState();
        if(gamepad1.aWasPressed())
            masterController.setTMode(MasterController.tModes.rCentric);

        if(gamepad1.dpadUpWasPressed())
            masterController.addSpeed(true);
        if(gamepad1.dpadDownWasPressed())
            masterController.addSpeed(false);

        if(masterController.getSOn()){
            gamepad1.rumble(100);
        }

        if(masterController.atTarget() && masterController.sNuN())
            gamepad2.rumble(100);


        if(gamepad1.rightStickButtonWasPressed())
            masterController.sweepSlowMode();

        masterController.drive(true, gamepad1);

        follower.update();
        masterController.update();
        masterController.debug(multipleTelemetry);
        multipleTelemetry.update();
    }
}
