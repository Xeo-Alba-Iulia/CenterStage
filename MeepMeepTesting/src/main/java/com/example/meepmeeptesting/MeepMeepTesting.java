package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPos = new Pose2d(-37, -60, Math.toRadians(90));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15.7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                                .lineToLinearHeading(new Pose2d(-33,-32,Math.toRadians(90-20)))
                                .back(5)
                                .strafeLeft(20)
                                .lineToLinearHeading(new Pose2d(-35, -10, Math.toRadians(-180)))
                                .back(70)
                                .lineToLinearHeading(new Pose2d(46, -40, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(26,-10,Math.toRadians(180)))
                                .forward(80)
                                .strafeLeft(5)
                                .lineTo(new Vector2d(-48,-10))
                                .back(75)
                                .lineToLinearHeading(new Pose2d(46, -40, Math.toRadians(180)))
                                .strafeRight(20)
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