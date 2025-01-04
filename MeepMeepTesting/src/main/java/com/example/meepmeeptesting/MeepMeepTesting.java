package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        RoadRunnerBotEntity RedObservation1214 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.25)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, -64,  Math.toRadians(90)))
                        .forward(18)
                        .back(4)
                        .lineTo(new Vector2d(36, -48))
                        .lineTo(new Vector2d(36, 0))
                        .turn(Math.toRadians(180))
                        .waitSeconds(1)
                        .strafeLeft(12)
                        .lineTo(new Vector2d(52,-60))
                        .build());


        RoadRunnerBotEntity BlueObservation1214 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.25)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, 64,  Math.toRadians(-90)))
                        .forward(18)
                        .back(4)
                        .lineTo(new Vector2d(-36, 48))
                        .lineTo(new Vector2d(-36, 0))
                        .turn(Math.toRadians(180))
                        .waitSeconds(1)
                        .strafeLeft(12)
                        .lineTo(new Vector2d(-52,60))
                        .build());

        RoadRunnerBotEntity RedNet = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.25)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36, -64.875,  Math.toRadians(180)))
                        .waitSeconds(2)
                        .lineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(45)))
                        .forward(-8)
                        .waitSeconds(1)

                        .strafeRight(8)
                        .splineToLinearHeading(new Pose2d(-48, -12, Math.toRadians(-90)), Math.toRadians(180))
                        .lineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(45)))
                        .forward(-8)

                        .splineToLinearHeading(new Pose2d(-58, -12, Math.toRadians(-90)), Math.toRadians(180))
                        .lineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(45)))
                        .forward(-8)

                        .splineToLinearHeading(new Pose2d(-58, -24.75, Math.toRadians(180)), Math.toRadians(180))
                        .lineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(45)))
                        .forward(-8)

                        .lineToLinearHeading(new Pose2d(-20, 11, Math.toRadians(0)))

                        .build());

        RoadRunnerBotEntity blueNet = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.25)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(36, 64,  Math.toRadians(0)))
                        .waitSeconds(2)
                        .lineToLinearHeading(new Pose2d(48, 48, Math.toRadians(135 * -1)))
                        .forward(-8 * -1 )
                        .waitSeconds(1)

                        .strafeRight(8)
                        .splineToLinearHeading(new Pose2d(48, 16, Math.toRadians(-270)), Math.toRadians(360))
                        .lineToLinearHeading(new Pose2d(48, 48, Math.toRadians(-135)))
                        .forward(-8)

                        .splineToLinearHeading(new Pose2d(58, 16, Math.toRadians(-270)), Math.toRadians(360))
                        .lineToLinearHeading(new Pose2d(48, 48, Math.toRadians(-135)))
                        .forward(-8)

                        .splineToLinearHeading(new Pose2d(58, 25.75, Math.toRadians(0)), Math.toRadians(360))
                        .lineToLinearHeading(new Pose2d(48, 48, Math.toRadians(-135)))
                        .forward(-8)

                        .lineToLinearHeading(new Pose2d(20, -11, Math.toRadians(180)))

                        .build());

        RoadRunnerBotEntity redObservation = new DefaultBotBuilder(meepMeep)
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

        RoadRunnerBotEntity blueObservation = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.25)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-12, 64,  Math.toRadians(-90)))
                        .waitSeconds(2)
                        .lineToLinearHeading(new Pose2d(0, 30, Math.toRadians(-90)))
                        .waitSeconds(1)
                        .back(12)
                        .strafeRight(16)
                        .splineToLinearHeading(new Pose2d(-48, 12, Math.toRadians(-270)), Math.toRadians(-470))
                        .forward(48)

                        .splineToLinearHeading(new Pose2d(-56, 12, Math.toRadians(-270)), Math.toRadians(180))
                        .forward(48)

                        .splineToLinearHeading(new Pose2d(-64.875, 12, Math.toRadians(-270)), Math.toRadians(180))
                        .forward(48)

                        .lineToLinearHeading(new Pose2d(48, 48, Math.toRadians(-135)))
                        .waitSeconds(2)
                        .lineToLinearHeading(new Pose2d(20, 11, Math.toRadians(180)))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
               // .addEntity(BlueObservation1214)
                //.addEntity(RedObservation1214)
                .addEntity(RedNet)
                .addEntity(blueNet)
                .addEntity(redObservation)
                .addEntity(blueObservation)
                .start();
    }
}