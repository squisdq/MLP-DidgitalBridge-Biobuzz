package org.firstinspires.ftc.teamcode.bobot.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.bobot.Utils.MLPLut;

@Config
public class ShooterController {
    private final Motor sMotor;
    private final ServoEx bServo, hServo;
    private final PIDFController controller;
    private final MLPLut sLut, hLut;

    public static double kP, kF;

    private double cSpeed, tSpeed;
    private double bPos, hPos;

    private final double fSpeed = 0.00;
    private final double fPos = 0.00;
    private final double blockPos = 0.00;
    private final double openPos = 0.00;

    private double distance = 0.00;

    private bStates bState = bStates.block;
    private sModes sMode = sModes.lut;

    public ShooterController(HardwareMap hMap){
        sMotor = new Motor(hMap, "sMotor");
        bServo = new ServoEx(hMap, "bServo");
        hServo = new ServoEx(hMap, "hServo");

        controller = new PIDFController(kP, 0, 0, kF);
        sLut = MLPLut.builder()
                .add(0, 0)
                .add(0, 0)
                .add(0, 0)
                .allowExtrapolation(false)
                .build();

        hLut = MLPLut.builder()
                .add(0, 0)
                .add(0, 0)
                .add(0, 0)
                .allowExtrapolation(false)
                .build();

        cSpeed = 0;
        tSpeed = 0;

        bPos = blockPos;
        hPos =  0;
    }

    public void setDistance(double distance){
        this.distance = distance;
    }

    private void setSMode(sModes sState){
        this.sMode = sState;
    }

    public void setManual(){
        setSMode(sModes.manual);
    }

    public void setLut(){
        setSMode(sModes.lut);
    }

    public void setFixed(){
        setSMode(sModes.fixed);
    }

    public void switchSMode(){
        if(sMode != sModes.lut) {
            setLut();
        } else
            setFixed();
    }

    public void setHPos(double hPos){
        this.hPos = hPos;
    }

    public void setBPos(double bPos){
        this.bPos = bPos;
    }

    public void setBState(bStates bState){
        this.bState = bState;

        if(bState == bStates.block){
            setBPos(blockPos);
        } else
            setBPos(openPos);
    }

    public void setSpeed(double tSpeed){
        this.tSpeed = tSpeed;
    }

    public boolean atTarget(){
        return Math.abs(tSpeed - cSpeed) <= 40;
    }

    public void update(){
        cSpeed = sMotor.getCorrectedVelocity();
        if(sMode == sModes.lut){
            setSpeed(sLut.get(distance));
            setHPos(hLut.get(distance));
        } else if(sMode == sModes.fixed){
            setSpeed(fSpeed);
            setHPos(fPos);
        } else if(sMode == sModes.manual){
            setSpeed(tSpeed);
            setHPos(hPos);
        }

        controller.setPIDF(kP, 0, 0, kF);
        double power = controller.calculate(cSpeed, tSpeed);
        sMotor.set(power);

        bServo.set(bPos);
        hServo.set(hPos);
    }

    public void debug(MultipleTelemetry telemetry){
        telemetry.addLine("----- SHOOTER CONTROLLER -----");
        telemetry.addData("SC: atTarget: ", atTarget());
        telemetry.addData("SC: cSpeed: ", cSpeed);
        telemetry.addData("SC: tSpeed: ", tSpeed);
        telemetry.addData("SC: sMode: ", sMode);
        telemetry.addData("SC: bPos: ", bPos);
        telemetry.addData("SC: hPos: ", hPos);
        telemetry.addData("SC: bState: ", bState);
    }

    public enum bStates{
        block, open;
    }

    public enum sModes {
        lut, fixed, manual;
    }
}