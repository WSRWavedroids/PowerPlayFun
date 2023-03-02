package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AprilTags.MayFlowers;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Isolation Arm Point", group = "RR")
public class IsolationPointRR extends AutonomousPLUS {
    @Override
    public void runOpMode() {
        //Runs the runOpMode() function in AutonomousPLUS because this is an extension of it
        super.runOpMode();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Basic procedure for setting up AprilTags recognition
        org.firstinspires.ftc.teamcode.Autonomous.AprilTags.MayFlowers MayFlowers = new MayFlowers();

        //MayFlowers.initCamera(hardwareMap, telemetry, this);

        //while (!isStarted() && !isStopRequested()) {
            //MayFlowers.DEATHLOOP(MayFlowers.aprilTagDetectionPipeline);
            //telemetry.addData("Zone", robot.parkingZone);
           // telemetry.update();
           // idle();
      //  }

        //Control hub is offset 4.5 inches in x and 3 inches in y
        //Establishing the hardware map
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Setting starting position
        Pose2d startPose = new Pose2d(-36, 63, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                //Move forward 37 inches (-36, 63) -> (-36, 26) and rotate 90 degrees to the right
                .addDisplacementMarker(() -> {
                    robot.openAndCloseClaw(1);
                    sleep(50);
                    robot.slide.setPower(-0.3);
                    sleep(50);
                })

                .lineToSplineHeading(new Pose2d(-36,24.5, Math.toRadians(180)))

                //Arm witchery *DISPLACEMENT MARKER HERE*
                .addDisplacementMarker(() -> {
                    robot.slide.setPower(-0.45);
                    sleep(300);
                })

                //Move forward 3 inches (-36, 36) -> (-36, 33)
                .forward(7)

                //Place the cone
                .addDisplacementMarker(() -> {
                    sleep(50);
                    robot.moveArm("Down");
                    robot.openAndCloseClaw(0.5);
                    sleep(100);
                })

                //Move backward 3 inches (-36, 33) -> (-36, 36)
                .back(7)


                //Move left 20 inches (-36, 36) -> (-36, 16)
                .lineToLinearHeading(new Pose2d(-36, 13,Math.toRadians(180)))

                //Move forward 28 inches (-36, 16) -> (-64, 16)
                .lineToLinearHeading(new Pose2d(-65, 13,Math.toRadians(180)))

                .addDisplacementMarker(() -> {
                    robot.slide.setPower(-0.47);
                    sleep(125);
                })

                //Creep forward 2 inches (-64, 16) -> (-66, 16)
                .forward(1)

                //Pick up a cone *DISPLACEMENT MARKER HERE*
                .addDisplacementMarker(() -> {
                    robot.openAndCloseClaw(1);
                    robot.slide.setPower(-0.5);
                    sleep(175);
                })

                //Creep backward 2 inches (-66, 16) -> (-64, 16)
                .back(3)


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

