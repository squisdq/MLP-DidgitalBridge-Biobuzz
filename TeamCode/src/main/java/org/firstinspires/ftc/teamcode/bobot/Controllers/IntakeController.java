package org.firstinspires.ftc.teamcode.bobot.Controllers;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class IntakeController {
    private Motor iMotor1, iMotor2;
    private ServoEx armServo;

    private double power;
    private double armPos;
    private final double upPos = 0;
    private final double flowerPos = 0.93;

    private armStates state;

    public IntakeController(HardwareMap hMap){
        iMotor1 = new Motor(hMap, "iMotor1");
        iMotor2 = new Motor(hMap, "iMotor2");
        iMotor2.setInverted(true);

        armServo = new ServoEx(hMap, "armServo");
//        armServo.setInverted(true);
//        armServo.setPwm(new PwmControl.PwmRange(500, 5000));

        power = 0;
        armPos = upPos;
        state = armStates.up;
    }

    public void initServo(){armServo.set(armPos);}

    public void setPower(double power){
        this.power = power;
    }

    public void setPos(double armPos){
        this.armPos = armPos;
    }

    public void setState(armStates state){
        this.state = state;

        if(state == armStates.up)
            setPos(upPos);
        else
            setPos(flowerPos);
    }

    public void switchState(){
        if(state == armStates.up)
            setState(armStates.flower);
        else
            setState(armStates.up);
    }

    public void update(){
        iMotor1.set(power);
        iMotor2.set(power);

        armServo.set(armPos);
    }

    public void debug(MultipleTelemetry telemetry){
        telemetry.addLine("----- INTAKE CONTROLLER -----");
        telemetry.addData("IC: power: ", power);
        telemetry.addData("IC: armPos: ", armPos);
        telemetry.addData("IC: state: ", state);
    }

    public enum armStates{
        up, flower
    }
}