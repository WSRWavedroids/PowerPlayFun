package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AprilTags.MayFlowers;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;
import org.firstinspires.ftc.teamcode.Autonomous.Falafel;

@Autonomous(group = "Blue", name = "Blue A2 Start (Blue Terminal)")
public class BlueA2Start extends AutonomousPLUS {
    @Override
    public void runOpMode() {

        Falafel Falafel = new Falafel();
        MayFlowers MayFlowers = new MayFlowers();
        super.runOpMode();
        robot.startingPosition = "Blue A2";
        sleepTime = 400;

        MayFlowers.initCamera(hardwareMap, telemetry, this);

        while (!isStarted() && !isStopRequested()) {
            MayFlowers.DEATHLOOP(MayFlowers.aprilTagDetectionPipeline);
            telemetry.addData("Zone", robot.parkingZone);
            telemetry.update();
            idle();
        }

        //Do this to pass inspection.o
        waitForStart();

        //MayFlowers.AprilTagsUpdate();

        robot.openAndCloseClaw(1);
        prepareNextAction(300);
        sleepTime = 400;
        moveArm("Up", 0.75);
        //moveArmE("Up", 40);
        moveRobotForward(1500, 0);
        moveRobotBackward(300, 0);
        moveRobotLeft(1825, 0);
        sleepTime = (1600);//was 1650
        moveArm("Up",0.75);
        speed = 0.25;
        moveRobotForward(275, 0);
        moveArm("Down",0.75);
        //sleep(50);
        robot.slide.setPower(0.1);
        robot.openAndCloseClaw(0.78);
        //prepareNextAction(25);
        moveRobotBackward(200, 0);
        //prepareNextAction(100);
        speed = 0.5;

        if (robot.parkingZone == 1){

           moveRobotRight(600, 0);

        } else if (robot.parkingZone == 2){

           moveRobotRight(1900, 0);

        } else if (robot.parkingZone == 3){

            moveRobotRight(3200, 0);

        } else {

            moveRobotRight(600, 0);

        }

        /*moveRobotForward(740, 50);
        robot.openAndCloseClaw(0.3);
        moveArmE("Up", 20);

        //Falafel.turnToHeading(0.5, -90);
        turnRobotRight(800,50);
        moveRobotForward(2000,50);
        //Falafel.driveStraight(0.5, 24, 0);
        moveRobotForward(50, 50);
        robot.openAndCloseClaw(0);
        moveRobotBackward(100, 50);

        //Turntable cycling option
        moveTurntable("Left", 600);
        moveRobotBackward(500,50);
        moveArmE("Up", 1000);
        moveRobotLeft(100,50);
        robot.openAndCloseClaw(0.3);
        moveRobotRight(100, 50);
        moveArmE("Down", 1000);
        moveTurntable("Right", 600);


        //Drivetrain turn cycling option
        //Falafel.turnToHeading(0.5,90);
        turnRobotLeft(800,50);
        moveRobotLeft(500, 50);
        moveArmE("Up", 1000);
        moveRobotForward(100, 50);
        robot.openAndCloseClaw(0.3);
        moveRobotBackward(100, 50);
        moveArmE("Down", 1000);
        */
    }
}

