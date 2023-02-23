package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBlueA5 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(36, 63, Math.toRadians(-90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(520, 30, 5.070000171661377, Math.toRadians(60), 13.02)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)

                                //Move forward 27 inches (-36, 63) -> (-36, 36)
                                //Move left 36 inches (-36, 36) -> (0, 36)
                                .splineTo(new Vector2d(24,36), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(8, 24, Math.toRadians(-180)), Math.toRadians(0))
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
                                .lineToLinearHeading(new Pose2d(12, 12, Math.toRadians(-90)))
                                //.splineTo(new Vector2d(-12, 12), Math.toRadians(-180))
                                //Move forward 40(?) inches (-12, 12) -> (-52, 12)
                                //Turn 90 degrees to the right *ADJUST HEADING*
                                .lineToLinearHeading(new Pose2d(52, 12, Math.toRadians(0)))
                                //Pick up a cone *DISPLACEMENT MARKER HERE*
                                .addDisplacementMarker(() -> {
                                    //robot.openAndCloseClaw(0);
                                    //robot.moveArm("Up");
                                })
                                //1. Move backward 28 inches (-52, 12) -> (-24, 12)
                                //2. Turn 90 degrees to left *ADJUST HEADING*
                                .lineToLinearHeading(new Pose2d(24, 12, Math.toRadians(-90)))
                                //3. Put cone on high pole *DISPLACEMENT MARKER HERE*
                                .addDisplacementMarker(() -> {
                                    //robot.moveArm("Down");
                                    //robot.openAndCloseClaw(0.3);
                                })
                                //4. Turn 90 degrees to right *ADJUST HEADING*
                                //5. Move forward 28 inches (-24, 12) -> (-52, 12)
                                .lineToLinearHeading(new Pose2d(52, 12, Math.toRadians(0)))
                                //6. Pick up a cone *DISPLACEMENT MARKER HERE*
                                .addDisplacementMarker(() -> {
                                    //robot.openAndCloseClaw(0);
                                    //robot.moveArm("Up");
                                })

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();


    }
}


