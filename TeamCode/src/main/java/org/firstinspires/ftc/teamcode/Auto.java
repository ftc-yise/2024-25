 package org.firstinspires.ftc.teamcode;

 import com.acmerobotics.roadrunner.geometry.Pose2d;
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.teamcode.yise.ledLights;
 import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
 import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

 import org.firstinspires.ftc.teamcode.yise.LiftClass;

 import org.firstinspires.ftc.teamcode.yise.Parameters;

 import org.firstinspires.ftc.teamcode.yise.poseStorage;

 @Autonomous(name="SpecimenHangAuto", group="Linear Opmode")

 public class Auto extends LinearOpMode {

  int stateExternal = -1;

 //Initialize timer
 private ElapsedTime runtime = new ElapsedTime();

  public TrajectorySequence InitialBlockScore(SampleMecanumDrive drive, Pose2d startPose, LiftClass arm) {
   //Build the trajectory sequence
   TrajectorySequence mySequence = drive.trajectorySequenceBuilder(startPose)
           //Wait for however long drivers want before moving
           .waitSeconds(Parameters.WAIT)

           //Go to calculated position
           .strafeRight(9)
           .back(33)
           .build();

   //Return the built sequence so it can be run
   return mySequence;
  }

 //Attach SPECIMEN on High CHAMBER or SAMPLE on

 public TrajectorySequence BlockPickUp(SampleMecanumDrive drive, Pose2d startPose, LiftClass arm) {

  //Create all position variables that will be changed
  double heading;
  double x;
  double y;

  //Get the alliance color and starting side of truss
  //Calculate coordinates depending on prop location
  if (Parameters.allianceColor == Parameters.Color.RED) {
   //start position should be 12, -64, 90
   heading = -90;
   x = 32;
   y = -50;

  } else {
   //start position should be -12, 64, -90
   heading = 90;
   x = -32;
   y = 50;
  }

  //Build the trajectory sequence
  TrajectorySequence mySequence = drive.trajectorySequenceBuilder(startPose)
          //Go to calculated position
          .forward(6)
          .addTemporalMarker(0.75, () -> {
           // This marker runs two seconds into the trajectory
           arm.setShoulderPosition(0.3);
           arm.setElbowPosition(0.04);
           // Run your action in here!
          })
          .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(heading)))
          .forward(8)
          .build();

  //Return the built sequence so it can be run
  return mySequence;
 }

  //Navigating to and picking up Yellow or Team Sample
  public TrajectorySequence BlockScore(SampleMecanumDrive drive, Pose2d startPose, LiftClass arm, ledLights leds, int runs) {
  int DiectionalMulti = 1;

   if (Parameters.allianceColor == Parameters.Color.RED) {
    DiectionalMulti = -DiectionalMulti;
   }
   runs = runs * 4 * DiectionalMulti;
   TrajectorySequence blockScoreObservation = drive.trajectorySequenceBuilder(startPose)
           .waitSeconds(.1)
           .back(5)
           .addTemporalMarker(.3, () -> {
            // This example will run 50% of the way through the path, plus 0.1 seconds
            //arm.setShoulderPosition(0.3);
            arm.setLiftPosition(LiftClass.liftPosition.BASKET);
            leds.setLed(ledLights.ledStates.GRAB_Y);
           })

           .addTemporalMarker(0.7, () -> {
            // This example will run 50% of the way through the path, plus 0.1 seconds
            //arm.setShoulderPosition(0.3);
            arm.setLiftPower(0);
            leds.setLed(ledLights.ledStates.BLUE);
           })

           .splineToLinearHeading(new Pose2d(0 + runs, 31 * DiectionalMulti, Math.toRadians(90 * DiectionalMulti)), Math.toRadians(270))
           .waitSeconds(0.1)
           .back(2)
           .build();
    return blockScoreObservation;
  }

   //Navigating to and scoring Yellow or Team Sample
   public TrajectorySequence LodgeBlockIntoObservation(SampleMecanumDrive drive, Pose2d startPose, LiftClass arm) {
    //Create all position variables that will be changed
    double heading;
    double x;
    double y;

    //Get the alliance color and starting side of truss
    //Calculate coordinates depending on prop location
    if (Parameters.allianceColor == Parameters.Color.RED) {
     //start position should be 12, -64, 90
     heading = -90;
     x = 48;
     y = -12;

    } else {
     //start position should be -12, 64, -90
     heading = 90;
     x = -48;
     y = 12;
    }


   double blockX = 0;
    double blockY = 0;
    double blockHeading = 0;
    double blockTangent = 0;
    int runsX = 1;
    int runsY = 1;

    if (Parameters.allianceColor == Parameters.Color.RED) {
     //56, -12, Math.toRadians(-90), Math.toRadians(0)
     blockX = 56;
     blockY = -12;
     blockHeading = -90;
     blockTangent = 0;
    } else {
     blockX = -56;
     blockY = 12;
     blockHeading = 90;
     blockTangent = 180;

     runsX = -runsX;
    }

    runsX = runsX * 6;
    runsY = runsY * 10;

    TrajectorySequence lodgeBlocksIntoObservation = drive.trajectorySequenceBuilder(startPose)
            //Go to calculated position
            .forward(5)
            .strafeLeft(34)
            .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(heading)))
            .waitSeconds(0.1)
            .forward(40)

            .splineToLinearHeading(new Pose2d(blockX, blockY, Math.toRadians(blockHeading)), Math.toRadians(blockTangent))

            .forward(34.5)
            .addTemporalMarker(5.5, () -> {
             // This marker runs two seconds into the trajectory
             arm.setShoulderPosition(0.3);
             arm.setElbowPosition(0.04);
             // Run your action in here!
            })

            .splineToLinearHeading(new Pose2d(blockX + runsX, blockY, Math.toRadians(blockHeading)), Math.toRadians(blockTangent))
            .forward(39.5 + runsY)
            .build();

     return lodgeBlocksIntoObservation;
  }

//Navigating to parking pose
 public TrajectorySequence Parking(SampleMecanumDrive drive, Pose2d startPose) {
  TrajectorySequence parkRedBasket = drive.trajectorySequenceBuilder(startPose)
          .lineToLinearHeading(new Pose2d(-20, 11, Math.toRadians(0)))
          .build();
  TrajectorySequence parkRedObservation = drive.trajectorySequenceBuilder(startPose)
          .lineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(45)))
          .waitSeconds(0.5)
          .lineToLinearHeading(new Pose2d(-20, -11, Math.toRadians(0)))
          .build();
  TrajectorySequence parkBlueBasket = drive.trajectorySequenceBuilder(startPose)
          .lineToLinearHeading(new Pose2d(20, -11, Math.toRadians(180)))
          .build();
  TrajectorySequence parkBlueObservation = drive.trajectorySequenceBuilder(startPose)
          .lineToLinearHeading(new Pose2d(48, 48, Math.toRadians(-135)))
          .waitSeconds(0.5)
          .lineToLinearHeading(new Pose2d(20, 11, Math.toRadians(180)))
          .build();
  if (Parameters.autoConfig == Parameters.AutonomousConfig.OBSERVATION && Parameters.allianceColor == Parameters.Color.RED) {
   return parkRedObservation;
  } else if (Parameters.autoConfig == Parameters.AutonomousConfig.OBSERVATION && Parameters.allianceColor == Parameters.Color.BLUE) {
   return parkBlueObservation;
  } else if (Parameters.autoConfig == Parameters.AutonomousConfig.BASKET && Parameters.allianceColor == Parameters.Color.RED) {
   return parkRedBasket;
  } else {
   return parkBlueBasket;
  }
 }

  @Override
  public void runOpMode() {

   //Initialize RR
   SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
   LiftClass arm = new LiftClass(hardwareMap);
   poseStorage.currentPose = drive.getPoseEstimate();

   ledLights leds = new ledLights(hardwareMap);

   // Variables to store start position and heading
   double startX, startY, startHeading;

// Determine the start pose based on autoConfig and allianceColor
   if (Parameters.allianceColor == Parameters.Color.RED) {
    // Red Specimen Hang config
    //12, -64,  Math.toRadians(90)
    startX = 4.875;
    startY = -64;
    startHeading = -90;
   } else {
    // Blue Specimen Hang config
    //-12, 64,  Math.toRadians(-90)
    startX = -4.875;
    startY = 64;
    startHeading =90;
   }

// Create the start pose using the calculated startX, startY, and startHeading
   Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startHeading));
   drive.setPoseEstimate(startPose);

   leds.setLed(ledLights.ledStates.INIT);

   waitForStart();

   if (Parameters.allianceColor == Parameters.Color.RED) {
   leds.setLed(ledLights.ledStates.RED);
   } else {
    leds.setLed(ledLights.ledStates.BLUE);
   }

   TrajectorySequence InitialBlockScore = InitialBlockScore(drive, startPose, arm);

   TrajectorySequence LodgeBlockIntoObservation = LodgeBlockIntoObservation(drive, InitialBlockScore.end(), arm);

   TrajectorySequence score1stBlock = BlockScore(drive, LodgeBlockIntoObservation.end(), arm, leds, 0);

   TrajectorySequence pickUp1stBlock = BlockPickUp(drive, score1stBlock.end(), arm);

   TrajectorySequence score2ndBlock = BlockScore(drive, pickUp1stBlock.end(), arm, leds, 1);
   TrajectorySequence score3rdBlock = BlockScore(drive, score2ndBlock.end(), arm, leds, 2);


   TrajectorySequence park = Parking(drive, score3rdBlock.end());

   drive.followTrajectorySequence(InitialBlockScore);

   arm.setLiftPosition(LiftClass.liftPosition.BASKET);
   while (arm.getLiftPositionL() <= 400) {
    sleep(50); // Check every 100ms
   }


   arm.setShoulderPosition(0.3);
   arm.setElbowPosition(0);
   arm.setClawPosition(0);

   sleep(350);

   arm.setLiftPosition(LiftClass.liftPosition.HOME);
   while (arm.getLiftPositionL() >= 350) {
    sleep(25);
   }
   arm.setClawPosition(1);

   sleep(150);

   arm.setShoulderPosition(0.45);
   arm.setElbowPosition(1);

   drive.followTrajectorySequence(LodgeBlockIntoObservation);

   arm.setClawPosition(0);
   sleep(100);

   arm.setShoulderPosition(0.25);
   arm.setElbowPosition(0.2);
   sleep(100);

   drive.followTrajectorySequence(score1stBlock);

   arm.setShoulderPosition(0.3);
   arm.setElbowPosition(0);
   arm.setClawPosition(0);

   sleep(350);

   arm.setLiftPosition(LiftClass.liftPosition.HOME);
   while (arm.getLiftPositionL() >= 350) {
    sleep(25);
   }
   arm.setClawPosition(1);

   sleep(150);

   arm.setShoulderPosition(0.45);
   arm.setElbowPosition(1);

   drive.followTrajectorySequence(pickUp1stBlock);

   arm.setClawPosition(0);
   sleep(100);

   arm.setShoulderPosition(0.25);
   arm.setElbowPosition(0.2);
   sleep(100);

   drive.followTrajectorySequence(score2ndBlock);

   arm.setShoulderPosition(0.3);
   arm.setElbowPosition(0);
   arm.setClawPosition(0);

   sleep(350);

   arm.setLiftPosition(LiftClass.liftPosition.HOME);
   while (arm.getLiftPositionL() >= 350) {
    sleep(25);
   }
   arm.setClawPosition(1);

   sleep(150);

   arm.setShoulderPosition(0.45);
   arm.setElbowPosition(1);

   sleep(2000);

   /*drive.followTrajectorySequence(score2ndBlock);
   drive.followTrajectorySequence(driveTo3rdBlock);
   drive.followTrajectorySequence(score3rdBlock);
   drive.followTrajectorySequence(park);*/

   poseStorage.currentPose = drive.getPoseEstimate();
  }
 }