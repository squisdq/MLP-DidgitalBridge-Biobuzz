package org.firstinspires.ftc.teamcode.bobot.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class TurretController {
    private final ServoEx tServo1, tServo2;

    private double tAngle = 0, tPos = 0.5;

    public TurretController(HardwareMap hMap){
        tServo1 = new ServoEx(hMap, "tServo1");
        tServo1.setPwm(new PwmControl.PwmRange(500, 2500));
        tServo2 = new ServoEx(hMap, "tServo2");
        tServo2.setPwm(new PwmControl.PwmRange(500, 2500));

    }

    public void setAngle(double tAngle){
        tAngle = AngleUnit.normalizeDegrees(tAngle - 180);

        if(tAngle > 140)
            tAngle = 140;
        else if(tAngle < -140)
            tAngle = -140;

        this.tAngle = tAngle;
    }

    public void update(){
        tPos = (175 - tAngle) / 350.0;

        tServo1.set(tPos);
        tServo2.set(tPos);
    }

    public void debug(MultipleTelemetry telemetry){
        telemetry.addLine("----- TURRET CONTROLLER -----");
        telemetry.addData("tPos", tPos);
        telemetry.addData("tAngle", tAngle);
    }
}
