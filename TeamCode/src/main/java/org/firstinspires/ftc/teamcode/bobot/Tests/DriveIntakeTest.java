package org.firstinspires.ftc.teamcode.bobot.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bobot.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@TeleOp(group = "!1Tests")
public class DriveIntakeTest extends OpMode {

    Follower follower;
    IntakeController iController;

    public static double sk = 0.5, tk = 0.7;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        iController = new IntakeController(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.left_trigger <= 0.2)
            sk = 1;
        else
            sk = 0.5;

        iController.setPower(gamepad1.right_trigger);
        if(gamepad1.a)
            iController.switchState();

        follower.manual(
                -gamepad1.left_stick_y*sk,
                gamepad1.left_stick_x*sk,
                gamepad1.right_stick_x*tk*sk
        );

        follower.update();
        iController.update();
    }
}
