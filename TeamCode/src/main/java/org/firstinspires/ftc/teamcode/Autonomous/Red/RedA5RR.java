package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AprilTags.MayFlowers;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Robot;


@Config
@Autonomous(name = "Red A5 RoadRunner (Red Side, Blue Terminal)", group = "Red")
public class RedA5RR extends AutonomousPLUS {
    @Override
    public void runOpMode() {
        super.runOpMode();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        org.firstinspires.ftc.teamcode.Autonomous.AprilTags.MayFlowers MayFlowers = new MayFlowers();

        MayFlowers.initCamera(hardwareMap, telemetry, this);

        while (!isStarted() && !isStopRequested()) {
            MayFlowers.DEATHLOOP(MayFlowers.aprilTagDetectionPipeline);
            telemetry.addData("Zone", robot.parkingZone);
            telemetry.update();
            idle();
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(36, -63, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width

                //Move forward 37 inches (-36, 63) -> (-36, 26) and rotate 90 degrees to the right
                .lineToSplineHeading(new Pose2d(36,-25, Math.toRadians(0)))

                //Arm witchery *DISPLACEMENT MARKER HERE*
                .addDisplacementMarker(() -> {
                    robot.slide.setPower(-0.8);
                    sleep(300);
                })

                //Move forward 3 inches (-36, 36) -> (-36, 33)
                .forward(5)

                //Place the cone
                .addDisplacementMarker(() -> {
                    robot.moveArm("Down");
                    robot.openAndCloseClaw(0.3);
                    sleep(100);
                })

                //Move backward 3 inches (-36, 33) -> (-36, 36)
                .back(5)

                //Move left 20 inches (-36, 36) -> (-36, 16)
                .lineToLinearHeading(new Pose2d(36, -14,Math.toRadians(0)))

                //Move forward 28 inches (-36, 16) -> (-64, 16)
                .lineToLinearHeading(new Pose2d(66, -14,Math.toRadians(0)))

                .addDisplacementMarker(() -> {
                    robot.slide.setPower(-0.5);
                    sleep(200);
                })

                //Creep forward 2 inches (-64, 16) -> (-66, 16)
                .forward(1)

                //Pick up a cone *DISPLACEMENT MARKER HERE*
                .addDisplacementMarker(() -> {
                    robot.openAndCloseClaw(0);
                    robot.slide.setPower(-0.5);
                    sleep(300);
                })

                //Creep backward 2 inches (-66, 16) -> (-64, 16)
                .back(1)




                //1. Move backward 38 inches (-64, 12) -> (-26, 12)
                //2. Turn 90 degrees to right *ADJUST HEADING*
                .lineToLinearHeading(new Pose2d(26, -16,Math.toRadians(-90)))

                //3. Put cone on medium pole *DISPLACEMENT MARKER HERE*
                .addDisplacementMarker(() -> {
                    robot.moveArm("Down");
                    robot.openAndCloseClaw(0.3);
                })

                //4. Turn 90 degrees to left *ADJUST HEADING*
                //5. Move forward 38 inches (-24, 12) -> (-64, 12)
                .lineToLinearHeading(new Pose2d(64, -16,Math.toRadians(0)))

                //6. Pick up a cone *DISPLACEMENT MARKER HERE*
                .addDisplacementMarker(() -> {
                    robot.openAndCloseClaw(0);
                    robot.moveArm("Up");
                })
                .build();


        waitForStart();

        if (isStopRequested()) return;

        robot.openAndCloseClaw(0);
        drive.followTrajectorySequence(trajSeq);
        //Repeat cycleCone trajectory as much as possible
        //Get to AprilTags position (figure out where we will be at that point)

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}



