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
    private final Motor sMotor, csMotor;
    private final ServoEx bServo;
    private final PIDFController controller, concontroller;
    private final MLPLut sLut, hLut;

    public static double kP = 0.0017, kF = 0.00049;
    public static double ckP = 0.0004, ckF = 0.000378;

    private double cSpeed, ccSpeed, tSpeed, ctSpeed, aSpeed;
    private double bPos = 0.6;

    private final double fSpeed = 737;
    private final double blockPos = 0.34;
    private final double openPos = 0.53;

    public static double sK = 1.7;

    private double distance = 0.00;

    private bStates bState = bStates.block;
    private sModes sMode = sModes.lut;
    private boolean sOn;

    public ShooterController(HardwareMap hMap){
        sMotor = new Motor(hMap, "sMotor");
        sMotor.setInverted(true);
        bServo = new ServoEx(hMap, "bServo");
        bServo.setInverted(true);
        csMotor = new Motor(hMap, "csMotor");
        csMotor.setInverted(true);

        controller = new PIDFController(kP, 0, 0, kF);
        concontroller = new PIDFController(ckP, 0, 0, ckF);

        sLut = MLPLut.builder()
                .add(28, 675)
                .add(55, 728)
                .add(72, 890)
                .allowExtrapolation(true)
                .build();

        hLut = MLPLut.builder()
                .add(30, 1.7)
                .add(55, 1.7)
                .add(72, 1.3)
                .allowExtrapolation(true)
                .build();

        ccSpeed = 0;
        cSpeed = 0;
        tSpeed = 0;
        ctSpeed = 0;
        aSpeed = 0;

        bPos = blockPos;

        sOn = false;
    }

    public double getCSpeed(){return cSpeed;}

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

    public sModes getsMode(){return  sMode;}

    public void addSpeed(boolean direction){
        if(direction)
            aSpeed += 20;
        else
            aSpeed -= 20;
    }

    public double getTSpeed(){return tSpeed;}

    public void switchSMode(){
        if(sMode != sModes.lut) {
            setLut();
        } else
            setFixed();
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

    public void setsOn(boolean sOn) {
        this.sOn = sOn;
    }

    public void setSpeed(double tSpeed){
        this.tSpeed = tSpeed;
    }

    public boolean atTarget(){
        return Math.abs(tSpeed - cSpeed) <= 40;
    }

    public void update(){
        ctSpeed = tSpeed* sK;

        cSpeed = sMotor.getCorrectedVelocity();
        ccSpeed = csMotor.getCorrectedVelocity();
        if(sMode == sModes.lut){
            setSpeed(sLut.get(distance));
            sK = hLut.get(distance);
        } else if(sMode == sModes.fixed){
            setSpeed(fSpeed);
            sK = 1.7;
        } else if(sMode == sModes.manual){
            setSpeed(tSpeed);
            sK = sK;
        }

        controller.setPIDF(kP, 0, 0, kF);
        concontroller.setPIDF(ckP, 0, 0, ckF);
        double power = controller.calculate(cSpeed, tSpeed + aSpeed);
        double cPower = concontroller.calculate(ccSpeed, tSpeed*sK + aSpeed*sK);
        if(tSpeed == 0 || !sOn) {
            aSpeed = 0;
            sMotor.set(0);
            csMotor.set(0);
            tSpeed = 0;
        } else {
            sMotor.set(power);
            csMotor.set(cPower);
            sOn = true;
        }
        bServo.set(bPos);
    }

    public void debug(MultipleTelemetry telemetry){
        telemetry.addLine("----- SHOOTER CONTROLLER -----");
        telemetry.addData("SC: atTarget: ", atTarget());
        telemetry.addData("SC: cSpeed: ", cSpeed);
        telemetry.addData("SC: tSpeed: ", tSpeed);
        telemetry.addData("SC: ccSpeed: ", ccSpeed);
        telemetry.addData("SC: ctSpeed: ", ctSpeed);
        telemetry.addData("SC: aSpeed: ", aSpeed);
        telemetry.addData("SC: sMode: ", sMode);
        telemetry.addData("SC: bPos: ", bPos);
        telemetry.addData("SC: bState: ", bState);
        telemetry.addData("SC: sk: ", sK);

    }

    public enum bStates{
        block, open;
    }

    public enum sModes {
        lut, fixed, manual;
    }
}