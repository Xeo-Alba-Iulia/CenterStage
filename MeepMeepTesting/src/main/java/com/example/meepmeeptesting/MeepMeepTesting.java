package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPos = new Pose2d(12, 60, Math.toRadians(270));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15.71)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                                .lineToLinearHeading(new Pose2d(13, 35, Math.toRadians(360-30)))
                                .back(5)
                                .lineToLinearHeading(new Pose2d(24,60,Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(46,40,Math.toRadians(180)))
                                .strafeRight(19)
//                                .forward(100)
//                                .lineTo(new Vector2d(-54,36))
//                                .strafeLeft(7)
//                                .strafeRight(30)
//                                .back(100)
//                                .strafeLeft(19)
//                                .strafeRight(20)
                                .back(5)
                                .build());
        //rosu stanga centru sa se stieee

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}