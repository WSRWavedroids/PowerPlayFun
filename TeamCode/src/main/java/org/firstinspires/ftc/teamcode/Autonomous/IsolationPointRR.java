package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.AprilTags.MayFlowers;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Robot;



@Config
@Autonomous(name = "Isolation Arm Point", group = "RR")
public class IsolationPointRR extends OpMode {

    public SampleMecanumDrive drive;
    Robot robot = new Robot();
    AutonomousPLUS AP = new AutonomousPLUS();


    @Override
    public void init() {

        AP.makeItWork(hardwareMap, telemetry, this);
        robot.init(hardwareMap, telemetry, this);
        drive = new SampleMecanumDrive(hardwareMap);

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        //Basic procedure for setting up AprilTags recognition
        org.firstinspires.ftc.teamcode.Autonomous.AprilTags.MayFlowers MayFlowers = new MayFlowers();
    }

    @Override
    public void start() {

        //Control hub is offset 4.5 inches in x and 3 inches in y

        //Setting starting position
        Pose2d startPose = new Pose2d(-36, 63, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                //Move forward 37 inches (-36, 63) -> (-36, 26) and rotate 90 degrees to the right
                .addDisplacementMarker(() -> {
                    AP.slidePos = 400;
                    robot.openAndCloseClaw(1);
                })

                .lineToSplineHeading(new Pose2d(-36, 24.5, Math.toRadians(180)))

                //Arm witchery *DISPLACEMENT MARKER HERE*
                .addDisplacementMarker(() -> {
                    AP.slidePos = 1000;
                })

                //Move forward 3 inches (-36, 36) -> (-36, 33)
                .forward(7)

                //Place the cone
                .addDisplacementMarker(() -> {
                    AP.slidePos = 400;
                })

                //Move backward 3 inches (-36, 33) -> (-36, 36)
                .back(7)


                //Move left 20 inches (-36, 36) -> (-36, 16)
                .lineToLinearHeading(new Pose2d(-36, 13, Math.toRadians(180)))

                //Move forward 28 inches (-36, 16) -> (-64, 16)
                .lineToLinearHeading(new Pose2d(-65, 13, Math.toRadians(180)))


                //Creep forward 2 inches (-64, 16) -> (-66, 16)
                .forward(1)

                //Pick up a cone *DISPLACEMENT MARKER HERE*
                .addDisplacementMarker(() -> {
                    robot.openAndCloseClaw(1);
                    AP.slidePos = 100;
                })

                //Creep backward 2 inches (-66, 16) -> (-64, 16)
                .back(3)

                .addDisplacementMarker(() -> {
                    AP.slidePos = 1000;
                })
                
                .build();
        drive.followTrajectorySequenceAsync(trajSeq);
    }


        @Override
        public void loop() {

            drive.update();

            AP.armPID();

        }

}


