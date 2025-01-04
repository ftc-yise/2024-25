package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity botBlue = new DefaultBotBuilder(meepMeep)
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

        RoadRunnerBotEntity botRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-23, -70, 0))

                        //place block 1
                        .lineToLinearHeading(new Pose2d(-40, -40, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)))
                        .waitSeconds(6)

                        //pick up block 2
                        .lineToLinearHeading(new Pose2d(-40, -42, Math.toRadians(90)))
                        .forward(4)
                        .strafeLeft(18)
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

        RoadRunnerBotEntity botBluePark = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.25)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12, -64,  Math.toRadians(90)))
                        .strafeRight(16)
                        .splineToLinearHeading(new Pose2d(48, -12, Math.toRadians(-90)), Math.toRadians(-470))
                        .forward(48)

                        .splineToLinearHeading(new Pose2d(56, -12, Math.toRadians(-90)), Math.toRadians(0))
                        .forward(48)

                        .splineToLinearHeading(new Pose2d(64.875, -12, Math.toRadians(-90)), Math.toRadians(0))
                        .forward(48)

                        .lineToLinearHeading(new Pose2d(0, -30, Math.toRadians(90)))

                        .lineToLinearHeading(new Pose2d(58, -58, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(0, -30, Math.toRadians(90)))

                        .lineToLinearHeading(new Pose2d(58, -58, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(0, -30, Math.toRadians(90)))

                        .lineToLinearHeading(new Pose2d(58, -58, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(0, -30, Math.toRadians(90)))


                        .lineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(45)))
                        .waitSeconds(2)
                        .lineToLinearHeading(new Pose2d(-20, -11, Math.toRadians(0)))
                        .build());


        RoadRunnerBotEntity botRedPark = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-23, -70, 0))
                        .strafeRight(-30)
                        .turn(Math.toRadians(-90))
                        .strafeLeft(-17)
                        .forward(-30)
                        .turn(Math.toRadians(90))
                        .forward(16)
                        .waitSeconds(2)

                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(botBlue)
                .addEntity(botRed)
                .addEntity(botBluePark)
                .addEntity(botRedPark)
                .start();
    }
}