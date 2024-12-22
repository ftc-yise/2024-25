package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(23, 70, 0))
                        .lineToLinearHeading(new Pose2d(40, 40, Math.toRadians(-90)))
                        .strafeLeft(8)
                        .waitSeconds(2)
                        .strafeLeft(12)
                        .lineToLinearHeading(new Pose2d(55, 55, Math.toRadians(-135)))
                        .waitSeconds(8)
                        .lineToLinearHeading(new Pose2d(40, 40, Math.toRadians(270)))
                        .strafeLeft(18.50)
                        .waitSeconds(2)
                        .strafeLeft(-19)
                        .lineToLinearHeading(new Pose2d(55, 55, Math.toRadians(-135)))
                        .waitSeconds(8)
                        .lineToLinearHeading(new Pose2d(40, 40, Math.toRadians(-90)))
                        .forward(30)
                        .strafeRight(17)
                        .waitSeconds(4)

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}