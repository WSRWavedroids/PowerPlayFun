package org.firstinspires.ftc.teamcode.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;

@Autonomous(group = "Test", name = "Turn")
public class TurnTest extends AutonomousPLUS {
    @Override
    public void runOpMode() {

        super.runOpMode();
waitForStart();

        robot.openAndCloseClaw(0);
        prepareNextAction(10);
        sleepTime = 400;
        moveArm("Up", 0.75);
        moveRobotForward(1500, 1);
        moveRobotBackward(300, 1);
        moveRobotLeft(1825, 1);
        sleepTime = (1800);
        moveArm("Up",0.75);
        speed = 0.25;
        moveRobotForward(275, 1);
        moveArm("Down",0.75);
        sleep(50);
        robot.slide.setPower(0.1);
        robot.openAndCloseClaw(0.3);
        prepareNextAction(25);
        moveRobotBackward(150, 1);

        moveRobotRight(470, 1);
        moveRobotForward(1000,1);
        turnRobotRight(830, 1);
        sleepTime = (400);
        moveArm("Up",0.75);
        moveRobotForward(2500, 1);
        moveRobotBackward(1500,1);

        
    }}