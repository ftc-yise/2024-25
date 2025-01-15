 package org.firstinspires.ftc.teamcode;

 import com.acmerobotics.roadrunner.geometry.Pose2d;
 import com.acmerobotics.roadrunner.geometry.Vector2d;
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.teamcode.archived23_24SeaonCenterStage.yiseArchived.LedLights;
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

  public TrajectorySequence BlueObservationPlace(SampleMecanumDrive drive, Pose2d startPose, LiftClass arm) {
   final int[] state = {-1};


   TrajectorySequence mySequence = drive.trajectorySequenceBuilder(startPose)
           //Wait for however long drivers want before moving
           .waitSeconds(Parameters.WAIT)

           //Go to calculated position
           .back(20)
           .addDisplacementMarker(() -> {
            if (Parameters.autoConfig == Parameters.AutonomousConfig.OBSERVATION) {
             switch (state[0]) {
              case -1:  // Initialize the
               arm.setShoulderPosition(0.25);
               arm.setElbowPosition(0);
               state[0] = 0;
               break;
              case 0:  // Initialize the
               arm.setPulleyPosition(LiftClass.PulleyPosition.HOME);
               if (arm.getPulleyPositionL() <= 350) {
                state[0]++;
               }
               break;
              case 1:
               arm.setLiftPosition(LiftClass.liftPosition.BASKET);
               if (arm.getLiftPositionL() >= 300) { // Replace with your own position checking logic
                state[0]++;
               }
               break;
              case 2:
               arm.setPulleyPosition(LiftClass.PulleyPosition.BASKET);
               if (arm.getPulleyPositionR() >= 3000) { // Replace with your own position checking logic
                state[0]++;
               }
               break;
              case 3:
               arm.setShoulderPosition(1);
               arm.setElbowPosition(0.65);
               sleep(2000);
               arm.setClawPosition(1);
               state[0]++;
               break;
              case 4:
               arm.setPulleyPosition(LiftClass.PulleyPosition.HOME);
               if (arm.getPulleyPositionR() <= 750) {
                arm.setLiftPosition(LiftClass.liftPosition.HOME);
                if (arm.getLiftPositionR() <= 300) {
                 state[0] = -1;
                }
               }
               break;
             }
            }
            })
           .build();

   //Return the built sequence so it can be run
   return mySequence;
  }

  public TrajectorySequence BlueObservationGrab(SampleMecanumDrive drive, Pose2d startPose, LiftClass arm) {
   final int[] state = {-1};


   TrajectorySequence mySequence = drive.trajectorySequenceBuilder(startPose)
           //Wait for however long drivers want before moving
           .waitSeconds(Parameters.WAIT)

           //Go to calculated position
           .waitSeconds(2)
           .forward(4)
           .lineTo(new Vector2d(-30, 48))
           .lineTo(new Vector2d(-48, 18))
           .turn(Math.toRadians(180))
           .waitSeconds(1)
           .strafeRight(2)
           .lineTo(new Vector2d(-52,60))
           .build();

   //Return the built sequence so it can be run
   return mySequence;
  }

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

 public TrajectorySequence InitialBlockPickUp(SampleMecanumDrive drive, Pose2d startPose, LiftClass arm) {

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

  //Build the trajectory sequence
  TrajectorySequence mySequence = drive.trajectorySequenceBuilder(startPose)
          //Wait for however long drivers want before moving
          .waitSeconds(Parameters.WAIT)

          //Go to calculated position
          .forward(5)
          .strafeLeft(32)
          .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(heading)))
          .waitSeconds(0.25)
          .forward(44)
          .build();

  //Return the built sequence so it can be run
  return mySequence;
 }

  //Navigating to and picking up Yellow or Team Sample
  public TrajectorySequence BlockScore(SampleMecanumDrive drive, Pose2d startPose, LiftClass arm) {
  int DiectionalMulti = 1;

   if (Parameters.allianceColor == Parameters.Color.RED) {
    DiectionalMulti = -DiectionalMulti;
   }
   TrajectorySequence blockScoreObservation = drive.trajectorySequenceBuilder(startPose)
           .waitSeconds(.1)
           .back(2)
           .splineToLinearHeading(new Pose2d(0, 31 * DiectionalMulti, Math.toRadians(90 * DiectionalMulti)), Math.toRadians(270))
           .build();
    return blockScoreObservation;
  }

   //Navigating to and scoring Yellow or Team Sample
   public TrajectorySequence LodgeBlockIntoObservation(SampleMecanumDrive drive, Pose2d startPose, LiftClass arm, int runsX) {
    double blockX = 0;
    double blockY = 0;
    double blockHeading = 0;
    double blockTangent = 0;
    double runsY = 0;

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

    runsX = runsX * 4;

    TrajectorySequence blockPickUpObservation = drive.trajectorySequenceBuilder(startPose)
            .splineToLinearHeading(new Pose2d(blockX + runsX, blockY, Math.toRadians(blockHeading)), Math.toRadians(blockTangent))
            .forward(39.5 + runsY)
            .build();

     return blockPickUpObservation;
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

   LedLights leds = new LedLights(hardwareMap);

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

   leds.setLed(LedLights.ledStates.INIT);

   waitForStart();

   if (Parameters.allianceColor == Parameters.Color.RED) {
   leds.setLed(LedLights.ledStates.RED);
   } else {
    leds.setLed(LedLights.ledStates.BLUE);
   }

   TrajectorySequence InitialBlockScore = InitialBlockScore(drive, startPose, arm);

   TrajectorySequence InitialBlockPickUp = InitialBlockPickUp(drive, InitialBlockScore.end(), arm);

   TrajectorySequence SecondBlockPickUp = LodgeBlockIntoObservation(drive, InitialBlockPickUp.end(), arm, 0);
   TrajectorySequence ThirdBlockPickup = LodgeBlockIntoObservation(drive, SecondBlockPickUp.end(), arm, 1);

   TrajectorySequence score1stBlock = BlockScore(drive, ThirdBlockPickup.end(), arm);
   TrajectorySequence score2ndBlock = BlockScore(drive, score1stBlock.end(), arm);
   TrajectorySequence score3rdBlock = BlockScore(drive, score2ndBlock.end(), arm);

   TrajectorySequence park = Parking(drive, score3rdBlock.end());

   drive.followTrajectorySequence(InitialBlockScore);

   arm.setLiftPosition(LiftClass.liftPosition.BASKET);
   while (arm.getLiftPositionL() <= 400) {
    sleep(50); // Check every 100ms
   }


   arm.setShoulderPosition(0.3);
   arm.setElbowPosition(0);
   arm.setClawPosition(0);

   sleep(2000);

   arm.setLiftPosition(LiftClass.liftPosition.HOME);
   while (arm.getLiftPositionL() >= 375) {
    sleep(50);
   }
   arm.setClawPosition(1);

   sleep(250);

   arm.setShoulderPosition(0.45);
   arm.setElbowPosition(1);

   drive.followTrajectorySequence(InitialBlockPickUp);
   drive.followTrajectorySequence(SecondBlockPickUp);

   arm.setShoulderPosition(0.25);
   arm.setElbowPosition(0);

   drive.followTrajectorySequence(ThirdBlockPickup);
   arm.setClawPosition(0);
   sleep(250);
   drive.followTrajectorySequence(score1stBlock);
   /*drive.followTrajectorySequence(score2ndBlock);
   drive.followTrajectorySequence(driveTo3rdBlock);
   drive.followTrajectorySequence(score3rdBlock);
   drive.followTrajectorySequence(park);*/

   poseStorage.currentPose = drive.getPoseEstimate();
  }
 }