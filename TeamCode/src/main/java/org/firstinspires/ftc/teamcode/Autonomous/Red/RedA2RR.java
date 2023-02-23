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
@Autonomous(name = "Red A2 RoadRunner (Red Side, Red Terminal)", group = "Red")
public class RedA2RR extends AutonomousPLUS {
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

        Pose2d startPose = new Pose2d(-36, -63, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width

                //Move forward 27 inches (-36, 63) -> (-36, 36)
                //Move left 36 inches (-36, 36) -> (0, 36)
                .splineTo(new Vector2d(-24,-36), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-8, -24, Math.toRadians(180)), Math.toRadians(0))
                //Arm witchery *DISPLACEMENT MARKER HERE*
                .addDisplacementMarker(() -> {
                    //robot.moveArm("Up");
                })
                //Move forward 2 inches (0, 36) -> (0, 34)
                .forward(2)

                .addDisplacementMarker(() -> {
                    //robot.moveArm("Down");
                    //robot.openAndCloseClaw(0.3);
                })
                //Move backward 2 inches (0, 34) -> (0, 36)
                .back(2)
                //Move right 12 inches (0, 36) -> (-12, 36)
                //Move forward 24 inches (-12, 36) -> (-12, 12)
                .lineToLinearHeading(new Pose2d(-12, -12,Math.toRadians(90)))
                //.splineTo(new Vector2d(-12, 12), Math.toRadians(-180))
                //Move forward 40(?) inches (-12, 12) -> (-52, 12)
                //Turn 90 degrees to the right *ADJUST HEADING*
                .lineToLinearHeading(new Pose2d(-52, -12,Math.toRadians(-180)))
                //Pick up a cone *DISPLACEMENT MARKER HERE*
                .addDisplacementMarker(() -> {
                    //robot.openAndCloseClaw(0);
                    //robot.moveArm("Up");
                })
                //1. Move backward 28 inches (-52, 12) -> (-24, 12)
                //2. Turn 90 degrees to left *ADJUST HEADING*
                .lineToLinearHeading(new Pose2d(-24, -12,Math.toRadians(90)))
                //3. Put cone on high pole *DISPLACEMENT MARKER HERE*
                .addDisplacementMarker(() -> {
                    //robot.moveArm("Down");
                    //robot.openAndCloseClaw(0.3);
                })
                //4. Turn 90 degrees to right *ADJUST HEADING*
                //5. Move forward 28 inches (-24, 12) -> (-52, 12)
                .lineToLinearHeading(new Pose2d(-52, -12,Math.toRadians(-180)))
                //6. Pick up a cone *DISPLACEMENT MARKER HERE*
                .addDisplacementMarker(() -> {
                    //robot.openAndCloseClaw(0);
                    //robot.moveArm("Up");
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


