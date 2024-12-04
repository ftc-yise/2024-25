package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity RedNet = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.25)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36, -64.875,  Math.toRadians(180)))
                        .waitSeconds(2)
                        .lineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(-135)))
                        .build());

        RoadRunnerBotEntity blueNet = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.25)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(36, 64,  Math.toRadians(0)))
                        .waitSeconds(2)
                        .lineToLinearHeading(new Pose2d(48, 48, Math.toRadians(45)))
                        .build());

        RoadRunnerBotEntity redObservation = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.25)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12, -64,  Math.toRadians(90)))
                        .waitSeconds(2)
                        .lineToLinearHeading(new Pose2d(0, -30, Math.toRadians(90)))
                        .build());

        RoadRunnerBotEntity blueObservation = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.25)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-12, 64,  Math.toRadians(-90)))
                        .waitSeconds(2)
                        .lineToLinearHeading(new Pose2d(0, 30, Math.toRadians(-90)))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(RedNet)
                .addEntity(blueNet)
                .addEntity(redObservation)
                .addEntity(blueObservation)
                .start();
    }
}