package org.firstinspires.ftc.teamcode.bobot.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bobot.Controllers.TurretController;

@Config
@TeleOp(group = "!1Tests")
public class TurretTest extends OpMode {
    MultipleTelemetry multipleTelemetry;
    TurretController tController;
    public static double tAngle;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void init() {
        tController = new TurretController(hardwareMap);
        tAngle = 0;

        multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    }

    @Override
    public void loop() {
        tController.setAngle(tAngle);
        tController.update();
        tController.debug(multipleTelemetry);
        multipleTelemetry.update();
    }
}
