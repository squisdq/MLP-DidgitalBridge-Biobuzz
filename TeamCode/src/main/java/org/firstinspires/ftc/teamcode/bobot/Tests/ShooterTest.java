package org.firstinspires.ftc.teamcode.bobot.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bobot.Controllers.ShooterController;

@Config
@TeleOp(group = "!1Tests")
public class ShooterTest extends OpMode {
    ShooterController sController;
    public static double tSpeed = 0;
    public double cSpeed;
    public static double bPos = 0.50;
    MultipleTelemetry multipleTelemetry;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void init() {
        sController = new ShooterController(hardwareMap);
        sController.setManual();
        sController.setsOn(true);

        multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    }

    @Override
    public void loop() {
        sController.setSpeed(tSpeed);
        sController.setBPos(bPos);
        cSpeed = sController.getCSpeed();

        sController.update();
        sController.debug(multipleTelemetry);
        multipleTelemetry.update();
    }
}
