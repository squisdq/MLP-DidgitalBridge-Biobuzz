package org.firstinspires.ftc.teamcode.bobot.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import static com.pedropathing.api.Paths.*;

import java.util.ArrayList;
import java.util.List;

@Config
public class LimelightController {

    private final Limelight3A ll;
    private final Follower follower;

    // Координаты шаров относительно робота (мм), полученные из llpython
    private final List<double[]> ballsRobotRelative = new ArrayList<>();
    private int ballsFound = 0;

    // Перевод в дюймы (координатная сетка поля Pedro Pathing)
    public static double MM_TO_FIELD_UNITS = 1.0 / 25.4;

    public LimelightController(HardwareMap hMap, Follower follower) {
        ll = hMap.get(Limelight3A.class, "ll");
        this.follower = follower;
    }

    public void init() {
        ll.pipelineSwitch(0);
        ll.start();
    }

    public void switchPipeline(int index) {
        ll.pipelineSwitch(index);
    }

    public int getCurrentPipeline() {
        return ll.getStatus() != null ? ll.getStatus().getPipelineIndex() : -1;
    }

    public void update() {
        LLResult result = ll.getLatestResult();

        if (result == null || !result.isValid()) {
            ballsFound = 0;
            ballsRobotRelative.clear();
            return;
        }

        double[] llpython = result.getPythonOutput();
        if (llpython == null || llpython.length < 8) {
            ballsFound = 0;
            ballsRobotRelative.clear();
            return;
        }

        ballsRobotRelative.clear();
        ballsFound = (int) llpython[0];
        int count = Math.min(3, ballsFound);

        for (int i = 0; i < count; i++) {
            double x = llpython[1 + i * 2];
            double y = llpython[2 + i * 2];
            ballsRobotRelative.add(new double[]{x, y});
        }
    }

    public int getBallsFound() {
        return ballsFound;
    }

    public List<double[]> getBallsRobotRelative() {
        return ballsRobotRelative;
    }

    /**
     * Переводит точку из системы координат робота (X вперёд, Y влево, мм)
     * в глобальную позу на поле. По умолчанию подставляет текущий heading робота,
     * так как точный угол шара относительно поля определится при генерации пути.
     */
    private Pose robotRelativeToGlobal(double xRobotMM, double yRobotMM, Pose robotPose) {
        double xRobot = xRobotMM * MM_TO_FIELD_UNITS;
        double yRobot = yRobotMM * MM_TO_FIELD_UNITS;

        double heading = robotPose.heading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double globalX = robotPose.x() + (xRobot * cos - yRobot * sin);
        double globalY = robotPose.y() + (xRobot * sin + yRobot * cos);

        return new Pose(globalX, globalY, heading);
    }

    /**
     * Возвращает список найденных шаров в виде объектов Pose (глобальные координаты поля).
     */
    public List<Pose> getGlobalBallPoses() {
        Pose currentPose = follower.pose();
        List<Pose> poses = new ArrayList<>();
        for (double[] ball : ballsRobotRelative) {
            poses.add(robotRelativeToGlobal(ball[0], ball[1], currentPose));
        }
        return poses;
    }

    /**
     * Строит единую цепочку траекторий Path для проезда по всем обнаруженным шарам.
     */
    public Path buildPathToBalls() {
        List<Pose> globalPoses = getGlobalBallPoses();
        return buildPathThroughPoses(globalPoses);
    }

    /**
     * Динамический сборщик составного пути через массив Pose.
     * Рассчитывает направление движения тангенциально вектору линии между шарами.
     */
    public Path buildPathThroughPoses(List<Pose> targetPoses) {
        if (targetPoses == null || targetPoses.isEmpty()) {
            return null;
        }

        Pose currentPose = follower.pose();

        List<Pose> allPoses = new ArrayList<>();
        allPoses.add(currentPose);
        allPoses.addAll(targetPoses);

        List<Path> segments = new ArrayList<>();
        double previousHeading = currentPose.heading();

        for (int i = 0; i < allPoses.size() - 1; i++) {
            Pose from = allPoses.get(i);
            Pose to = allPoses.get(i + 1);

            // Рассчитываем угол движения от текущей позы к следующей
            double segmentHeading = Math.atan2(to.y() - from.y(), to.x() - from.x());

            // Создаем позы с выровненным по ходу движения углом (heading)
            Pose startSegmentPose = new Pose(from.x(), from.y(), previousHeading);
            Pose endSegmentPose = new Pose(to.x(), to.y(), segmentHeading);

            // Генерируем линию и применяем линейную интерполяцию направления
            Path segment = line(startSegmentPose, endSegmentPose)
                    .linear(startSegmentPose, endSegmentPose);

            segments.add(segment);
            previousHeading = segmentHeading;
        }

        // Связываем все сегменты через статический фабричный метод path() в единый Path
        return path(segments.toArray(new Path[0]));
    }

    public void driveToBalls() {
        update();
        Path pathChain = buildPathToBalls();
        if (pathChain != null) {
            follower.follow(pathChain);
        }
    }

    public void debug(MultipleTelemetry telemetry) {
        telemetry.addLine("----- LIMELIGHT CONTROLLER -----");
        telemetry.addData("Pipeline", getCurrentPipeline());
        telemetry.addData("Balls found", ballsFound);
        for (int i = 0; i < ballsRobotRelative.size(); i++) {
            double[] ball = ballsRobotRelative.get(i);
            telemetry.addData("Ball " + (i + 1) + " (robot, мм)", "X:%.1f Y:%.1f", ball[0], ball[1]);
        }
    }
}
