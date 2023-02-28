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
                .lineToSplineHeading(new Pose2d(-36,25, Math.toRadians(180)))

                //Arm witchery *DISPLACEMENT MARKER HERE*
                .addDisplacementMarker(() -> {
                    robot.slide.setPower(-0.9);
                    sleep(400);
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

