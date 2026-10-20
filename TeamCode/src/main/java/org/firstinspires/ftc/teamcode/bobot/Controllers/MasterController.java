package org.firstinspires.ftc.teamcode.bobot.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.bobot.Utils.MLPConfig;

@Config
public class MasterController {
    private ShooterController sController;
    private TurretController tController;
    private IntakeController iController;
    private Follower follower;

    private tModes tMode;
    private tGoal cGoal;
    private boolean sFlag, sOn, sf, slowMode;
    private final double oX = 2.24311024, oY = 0.00;
    private double hX, hY;
    private double tAngle;

    private final ElapsedTime shootTimer = new ElapsedTime();
    private boolean waitingToShoot = false;
    private static final double SHOOT_DELAY_MS = 100;
    public static double sPower = 0.55 ;
    private double smk;

    public static double aK= 0;

    public MasterController(HardwareMap hMap, Follower follower){
        sController = new ShooterController(hMap);
        tController = new TurretController(hMap);
        iController = new IntakeController(hMap);
        this.follower = follower;

        tMode = tModes.gCentric;
        cGoal = tGoal.right;
        sFlag = false;
        sOn = false;
        sf = false;
        slowMode = false;
    }

    public void initServo(){iController.initServo();}

    public void setSPower(double sPower){
        this.sPower = sPower;
    }

    public void resetPP(HardwareMap hMap){
        GoBildaPinpointDriver odo = hMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();
        odo.resetDeviceConfigurationForOpMode();}

    public void intake(){
        iController.setPower(1);
    }

    public void stopIntake(){
        iController.setPower(0);
        sFlag = false;
        waitingToShoot = false;
        sController.setBState(ShooterController.bStates.block);
    }

    public void outtake(){
        iController.setPower(-1);
    }

    public void shoot(){
        if(sController.getTSpeed() == 0)
            return;
        if(!sFlag) {
            iController.setPower(0);
            sController.setBState(ShooterController.bStates.open);
            shootTimer.reset();
            waitingToShoot = true;
            sFlag = true;
        }

        if(waitingToShoot && shootTimer.milliseconds() >= SHOOT_DELAY_MS){
            iController.setPower(sPower);
            waitingToShoot = false;
        }
    }

    public boolean atTarget(){
        return sController.atTarget();
    }

    public void afterShoot(){
        sFlag = false;
        waitingToShoot = false;
        sController.setBState(ShooterController.bStates.block);
    }

    public void switchArmState(){
        iController.switchState();
    }

    public void setArmState(IntakeController.armStates armState){iController.setState(armState);}

    public void switchShooterMode(){
        sController.switchSMode();
    }

    public void setShooter(boolean isOn){
        sOn = isOn;
        sf = true;
        sController.setsOn(sOn);
    }

    public boolean getSOn(){return sOn;}

    public void toggleShooter(){
        setShooter(!sOn);
    }

    public void addSpeed(boolean direction){
        sController.addSpeed(direction);
    }

    public void setManual(){
        sController.setManual();
    }

    public void setSpeed(double speed){
        sController.setSpeed(speed);
    }

    public boolean sNuN(){return sController.getTSpeed() > 0;}

    public void resetPositionRight(){
        follower.setPose(MLPConfig.rightResetPose);
    }

    public void resetPositionLeft(){
        follower.setPose(MLPConfig.leftResetPose);
    }

    public void setTMode(tModes tMode){
        this.tMode = tMode;

        if(sController.getsMode() == ShooterController.sModes.manual)
            return;
        else {
            if (tMode == tModes.gCentric) {
                sController.setLut();
            } else {
                sController.setFixed();
            }
        }
    }

    public void sweepTMode(){
        if(tMode != tModes.gCentric)
            setTMode(tModes.gCentric);
        else
            setTMode(tModes.fCentric);
    }

    public void sweepGoal(){
        if(cGoal == tGoal.left) {
            cGoal = tGoal.right;
            hX = MLPConfig.rightHivePose.x();
            hY = MLPConfig.rightHivePose.y();
        }else {
            cGoal = tGoal.left;
            hX = MLPConfig.leftHivePose.x();
            hY = MLPConfig.leftHivePose.y();
        }
    }

    private void pickGoal(){
        double rY = follower.pose().y();

        if(rY < 72) {
            cGoal = tGoal.right;
            hX = MLPConfig.rightHivePose.x();
            hY = MLPConfig.rightHivePose.y();
        }else if(rY > 72){
            cGoal = tGoal.left;
            hX = MLPConfig.leftHivePose.x();
            hY = MLPConfig.leftHivePose.y();
        }
    }

    public double calculateGCentricAngle(){
        pickGoal();

        double heading = follower.pose().heading() + follower.velocity().omega * aK; // радианы
        double wX = follower.pose().x() + oX * Math.cos(heading) - oY * Math.sin(heading);
        double wY = follower.pose().y() + oX * Math.sin(heading) + oY * Math.cos(heading);

        double dx = hX - wX;
        double dy = hY - wY;

        double angleToGoal = Math.atan2(dy, dx);
        double normalizedHeading = AngleUnit.normalizeRadians(heading);

        return Math.toDegrees(AngleUnit.normalizeRadians(angleToGoal - normalizedHeading));
    }
    public double calculateFCentricAngle(){
        double heading = Math.toDegrees(AngleUnit.normalizeRadians(follower.pose().heading()));
        double tAngle;

        if(cGoal == tGoal.right)
            tAngle = 90;
        else
            tAngle = -90;

        return tAngle - heading;
    }

    public double calculateRCentricAngle(){
        return 0;
    }

    private double calculateDistance(){
        double heading = follower.pose().heading();
        double wX = follower.pose().x() + oX * Math.cos(heading) - oY * Math.sin(heading);
        double wY = follower.pose().y() + oX * Math.sin(heading) + oY * Math.cos(heading);

        double dx = hX - wX;
        double dy = hY - wY;

        return Math.hypot(dx, dy);
    }

    public void sweepSlowMode(){
        slowMode = !slowMode;
    }

    public void drive(boolean fieldCentric, Gamepad gamepad1){
        if(fieldCentric){
            DrivePowers powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y*smk,
                    -gamepad1.left_stick_x*smk,
                    -gamepad1.right_stick_x * 0.65*smk,
                    follower.pose().heading()
            );
            follower.manual(powers);
        } else
            follower.manual(
                    -gamepad1.left_stick_y * smk,
                    -gamepad1.left_stick_x * smk,
                    -gamepad1.right_stick_x*0.65*smk
            );
    }

    public void update(){
        if(slowMode)
            smk = 0.6;
        else
            smk = 1;

        if(tMode == tModes.gCentric) {
            tAngle = calculateGCentricAngle();
        }
        else if(tMode == tModes.fCentric) {
            tAngle = calculateFCentricAngle();
        }
        else if(tMode == tModes.rCentric) {
            tAngle = calculateRCentricAngle();
        }

        if(!sf){
            sController.setSpeed(0);
            sController.setDistance(0);
        }
        else {
            if (!sOn) {
                sController.setSpeed(0);
                sController.setDistance(0);
            } else {
                sController.setDistance(calculateDistance());
            }
        }
        tController.setAngle(tAngle);

        iController.update();
        if(sOn){tController.update();}
        sController.update();
    }

    public void debug(MultipleTelemetry telemetry){
        telemetry.addLine("Bismillah");
        telemetry.addLine("----- MASTER CONTROLLER -----");
        telemetry.addData("tMode", tMode);
        telemetry.addData("cGoal", cGoal);
        telemetry.addData("sFlag", sFlag);
        telemetry.addData("sOn", sOn);
        telemetry.addData("distance", calculateDistance());
        sController.debug(telemetry);
        iController.debug(telemetry);
        tController.debug(telemetry);
        telemetry.addLine("Bismillah");
    }

    public enum tModes{
        gCentric, fCentric, rCentric;
    }

    public enum tGoal{
        right, left;
    }
}