package org.firstinspires.ftc.teamcode.Autonomous.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPLUS;


@Autonomous(group = "Blue", name = "Arm Test")
public class ArmTest extends AutonomousPLUS {
    @Override
    public void runOpMode() {

        super.runOpMode();

        //Do this to pass inspection.
        waitForStart();

        moveArmE("Up", -2000);
        //sleep(2000);
        robot.openAndCloseClaw(1);

        //5 cones is 400 ticks
        //4 cones is 300 ticks
        //3 cones is 200 ticks

        //1400 ticks for low pole drop
        //2150 ticks for medium pole drop

    }
}

//blue a2 and red a5 zone 2 on medium pole and zone 3 small pole idk arm values
//red a2 blue a5 opposite

