package org.firstinspires.ftc.teamcode.bobot.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

@Config

public class MLPConfig {
    public static Pose startPose = new Pose(0, 0, 0);

    public static final Pose rightHivePose = new Pose(52, 60);
    public static final Pose leftHivePose = new Pose(65, 88);
    public static final Pose rightResetPose = new Pose(6.79, 8.28, Math.PI/2);
    public static final Pose leftResetPose = new Pose(144-6.79, 144-8.28, 3*Math.PI/2);
}