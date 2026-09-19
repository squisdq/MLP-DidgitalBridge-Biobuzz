package org.firstinspires.ftc.teamcode.bobot.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bobot.Controllers.IntakeController;

@Config
@TeleOp(group = "!1Tests")
public class IntakeTets extends OpMode {
    IntakeController iController;
    public static double power, armPos;
    MultipleTelemetry multipleTelemetry;

    @Override
    public void init() {
        iController = new IntakeController(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    }

    @Override
    public void loop() {
        iController.setPower(power);
        iController.setPos(armPos);

        iController.update();
        iController.debug(multipleTelemetry);
    }
}
