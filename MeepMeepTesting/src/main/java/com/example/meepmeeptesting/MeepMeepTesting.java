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

                        //place block 1
                        .lineToLinearHeading(new Pose2d(40, 40, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(55, 55, Math.toRadians(-135)))
                        .waitSeconds(6)

                        //pick up block 2
                        .lineToLinearHeading(new Pose2d(40, 40, Math.toRadians(270)))
                        .strafeLeft(18.50)
                        .waitSeconds(2)

                        //place block 2
                        .strafeLeft(-19)
                        .lineToLinearHeading(new Pose2d(55, 55, Math.toRadians(-135)))
                        .waitSeconds(6)

                        //park at submersible, level 1 hang
                        .lineToLinearHeading(new Pose2d(40, 40, Math.toRadians(-90)))
                        .forward(30)
                        .turn(Math.toRadians(-90))
                        .forward(16)
                        .waitSeconds(4)

                        .build());

        RoadRunnerBotEntity MyBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-23, -70, 0))

                        //place block 1
                        .lineToLinearHeading(new Pose2d(-40, -40, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)))
                        .waitSeconds(6)

                        //pick up block 2
                        .lineToLinearHeading(new Pose2d(-40, -40, Math.toRadians(90)))
                        .forward(4)
                        .strafeLeft(19)
                        .waitSeconds(2)

                        //place block 2
                        .strafeLeft(-19)
                        .lineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)))
                        .waitSeconds(6)

                        //park at submersible, level 1 hang
                        .lineToLinearHeading(new Pose2d(-40, -40, Math.toRadians(90)))
                        .forward(30)
                        .turn(Math.toRadians(-90))
                        .forward(16)
                        .waitSeconds(4)


                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(MyBot)
                .start();
    }
}