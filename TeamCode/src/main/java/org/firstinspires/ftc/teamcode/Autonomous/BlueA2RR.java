package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Config
@Autonomous(name = "Blue A2 RoadRunner", group = "drive")
public class BlueA2RR extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Starting position is -36, 63
        Pose2d startPose = new Pose2d(-36, 63, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory toFirstPole = drive.trajectoryBuilder(new Pose2d())
                .forward(27)
                .build();

        waitForStart();
        //Coordinate reference: x-axis is left and right between auto cone stacks, y-axis is up and down between substations
        //From this position... forward is subtracting from y, backward is adding to y, left is adding to x, right is subtracting from x

        //Move forward 27 inches (-36, 63) -> (-36, 36)
        //Move left 36 inches (-36, 36) -> (0, 36)
        //Move forward 2 inches (0, 36) -> (0, 34)
        //Arm witchery *DISPLACEMENT MARKER HERE*
        //Move backward 2 inches (0, 34) -> (0, 36)
        //Move right 12 inches (0, 36) -> (-12, 36)
        //Move forward 24 inches (-12, 36) -> (-12, 12)
        //Turn 90 degrees to the right *ADJUST HEADING*
        //Move forward 40(?) inches (-12, 12) -> (-52, 12)
        //Pick up a cone *DISPLACEMENT MARKER HERE*
        //1. Move backward 28 inches (-52, 12) -> (-24, 12)
        //2. Turn 90 degrees to left *ADJUST HEADING*
        //3. Put cone on high pole *DISPLACEMENT MARKER HERE*
        //4. Turn 90 degrees to right *ADJUST HEADING*
        //5. Move forward 28 inches (-24, 12) -> (-52, 12)
        //6. Pick up a cone *DISPLACEMENT MARKER HERE*
        //Repeat steps 1-6 as much as possible
        //Get to AprilTags position (figure out where we will be at that point)

        if (isStopRequested()) return;

        drive.followTrajectory(toFirstPole);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}

