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

 @Autonomous(name="Auto", group="Linear Opmode")

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
 //Attach SPECIMEN on High CHAMBER or SAMPLE on

 public TrajectorySequence InitialBlock(SampleMecanumDrive drive, Pose2d startPose, LiftClass arm) {

  //Create all position variables that will be changed
  double heading;
  double x;
  double y;
  int DirectionalMulti = 1;

  final int[] state = {-1};


  //Get the alliance color and starting side of truss
  //Calculate coordinates depending on prop location
  if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.BASKET) {
   //start position should be -36, -64.875, 180
   heading = -135;
   x = -48;
   y = -48;

  } else if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.OBSERVATION) {
   //start position should be 12, -64, 90
   heading = 90;
   x = 0;
   y = -30;
   DirectionalMulti = -1;

  } else if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.BASKET) {
   //start position should be 36, 64, 0
   heading = -135;
   x = 48;
   y = 48;


  } else {
   //start position should be -12, 64, -90
   heading = -90;
   x = 0;
   y = 30;
   DirectionalMulti = -1;
  }

  //Build the trajectory sequence
  TrajectorySequence mySequence = drive.trajectorySequenceBuilder(startPose)
          //Wait for however long drivers want before moving
          .waitSeconds(Parameters.WAIT)

          //Go to calculated position
          .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(heading + 180)))
          .waitSeconds(2)
          .forward(8* DirectionalMulti)
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
              if (arm.getLiftPositionR() <= 300){
               state[0] = -1;
              }
             }
             break;
            }
           } else {
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
              arm.setLiftPosition(LiftClass.liftPosition.SUBMERSIBLE);
              if (arm.getLiftPositionL() >= 300) { // Replace with your own position checking logic
               state[0]++;
              }
              break;
             case 2:
              arm.setPulleyPosition(LiftClass.PulleyPosition.SUBMERSIBLE);
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
              arm.setLiftPosition(LiftClass.liftPosition.HOME);
              if (arm.getLiftPositionL() <= 200) {
               arm.setClawPosition(0);
               state[0]++;
              }
               break;
               case 5:
                arm.setPulleyPosition(LiftClass.PulleyPosition.HOME);
                if (arm.getPulleyPositionR() <= 750) {
                 arm.setLiftPosition(LiftClass.liftPosition.HOME);
                 if (arm.getLiftPositionR() <= 300){
                  state[0] = -1;
                 }
                }
                break;
              }
           }
          })
          .back(8)
          .build();

  //Return the built sequence so it can be run
  return mySequence;
 }

  //Navigating to and picking up Yellow or Team Sample
  public TrajectorySequence BlockScore(SampleMecanumDrive drive, Pose2d startPose, LiftClass arm) {
   double scoreX = 0;
   double scoreY = 0;
   double scoreHeading = 0;

   final int[] state = {-1};

   if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.BASKET) {
    scoreX = -48;
    scoreY = -12;
    scoreHeading = -90;
   } else if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.BASKET) {
    scoreX = -48;
    scoreY = 12;
    scoreHeading = -270;
   }
   TrajectorySequence blockScoreObservation = drive.trajectorySequenceBuilder(startPose)
           .forward(-48)
           .build();

   TrajectorySequence blockScoreBasket = drive.trajectorySequenceBuilder(startPose)
           .lineToLinearHeading(new Pose2d(scoreX, scoreY, Math.toRadians(scoreHeading)))
           .back(5)
           .addDisplacementMarker(() -> {
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
              arm.setLiftPosition(LiftClass.liftPosition.SUBMERSIBLE);
              if (arm.getLiftPositionL() >= 300) { // Replace with your own position checking logic
               state[0]++;
              }
              break;
             case 2:
              arm.setPulleyPosition(LiftClass.PulleyPosition.SUBMERSIBLE);
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
              arm.setLiftPosition(LiftClass.liftPosition.HOME);
              if (arm.getLiftPositionL() <= 200) {
               arm.setClawPosition(0);
               state[0]++;
              }
              break;
             case 5:
              arm.setPulleyPosition(LiftClass.PulleyPosition.HOME);
              if (arm.getPulleyPositionR() <= 750) {
               arm.setLiftPosition(LiftClass.liftPosition.HOME);
               if (arm.getLiftPositionR() <= 300){
                state[0] = -1;
               }
              }
              break;
            }
           })
           .build();

   if (Parameters.autoConfig == Parameters.AutonomousConfig.OBSERVATION) {
    return blockScoreObservation;
   } else {
    return blockScoreBasket;
   }
  }

   //Navigating to and scoring Yellow or Team Sample
   public TrajectorySequence BlockPickup(SampleMecanumDrive drive, Pose2d startPose, LiftClass arm, int runs) {
    double blockX = 0;
    double blockY = 0;
    double blockHeading = 0;
    double blockTangent = 0;

    final int[] state = {-1};

    if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.BASKET) {
     blockX = -48;
     blockY = -12;
     blockHeading = -90;
     blockTangent = 180;
    } else if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.OBSERVATION) {
     blockX = 48;
     blockY = -12;
     blockHeading = -90;
     blockTangent = 0;
    } else if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.BASKET) {
     blockX = 48;
     blockY = 16;
     blockHeading = -270;
     blockTangent = 360;
     runs = -runs;
    } else if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.OBSERVATION) {
     blockX = -48;
     blockY = 12;
     blockHeading = -270;
     blockTangent = 180;

     runs = -runs;
    }

    if (runs == 2){
     if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.OBSERVATION){
      blockTangent = -470;
     } else if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.OBSERVATION) {
      blockTangent = -470;
     }
    }
    TrajectorySequence blockPickUpObservation = drive.trajectorySequenceBuilder(startPose)
            .splineToLinearHeading(new Pose2d(blockX + (8.4375 * runs), blockY, Math.toRadians(blockHeading + 180)), Math.toRadians(blockTangent))
            .back(5)
            .build();

    TrajectorySequence blockPickUpBasket = drive.trajectorySequenceBuilder(startPose)
            .splineToLinearHeading(new Pose2d(blockX + (8.4375 * runs), blockY, Math.toRadians(blockHeading)), Math.toRadians(blockTangent))
            .back(5)
            .addDisplacementMarker(() -> {
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
               state[0]++;
               break;
              case 2:
               arm.setPulleyPosition(LiftClass.PulleyPosition.SEARCH);
               if (arm.getPulleyPositionR() >= 3000) { // Replace with your own position checking logic
                state[0]++;
               }
               break;
              case 3:
               arm.setShoulderPosition(0.585);
               arm.setElbowPosition(0);
               sleep(2000);
               arm.setClawPosition(0);
               state[0]++;
               break;
              case 4:
               arm.setPulleyPosition(LiftClass.PulleyPosition.HOME);
               if (arm.getPulleyPositionR() <= 750) { // Replace with your own position checking logic
                state[0] = -1;
               }
             }
            })
            .build();

    if (Parameters.autoConfig == Parameters.AutonomousConfig.OBSERVATION) {
     return blockPickUpObservation;
    } else {
     return blockPickUpBasket;
    }
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
   if (Parameters.autoConfig == Parameters.AutonomousConfig.OBSERVATION && Parameters.allianceColor == Parameters.Color.RED) {
    // Red Observation config
    startX = -36;
    startY = -64.875;
    startHeading = Math.toRadians(90);
   } else if (Parameters.autoConfig == Parameters.AutonomousConfig.BASKET && Parameters.allianceColor == Parameters.Color.BLUE) {
    // Blue Observation config
    startX = 36;
    startY = 64;
    startHeading = Math.toRadians(-90);
   } else if (Parameters.autoConfig == Parameters.AutonomousConfig.BASKET && Parameters.allianceColor == Parameters.Color.RED) {
    // Red Net config
    startX = -36;
    startY = -64.875;
    startHeading = Math.toRadians(180);
   } else {
    // Blue Net config
    startX = 0;
    startY =64;
    startHeading = Math.toRadians(0);
   }

// Create the start pose using the calculated startX, startY, and startHeading
   Pose2d startPose = new Pose2d(0, 64, Math.toRadians(90));
   drive.setPoseEstimate(startPose);

   leds.setLed(LedLights.ledStates.INIT);

   waitForStart();

   if (Parameters.allianceColor == Parameters.Color.RED) {
   leds.setLed(LedLights.ledStates.RED);
   } else {
    leds.setLed(LedLights.ledStates.BLUE);
   }

   TrajectorySequence BlueOP = BlueObservationPlace(drive, startPose, arm);
   TrajectorySequence BlueOG = BlueObservationGrab(drive, BlueOP.end(), arm);

   TrajectorySequence InitialBlock = InitialBlock(drive, startPose, arm);
   TrajectorySequence driveTo1stBlock = BlockPickup(drive, InitialBlock.end(), arm, 0);
   TrajectorySequence score1stBlock = BlockScore(drive, driveTo1stBlock.end(), arm);

   TrajectorySequence driveTo2ndBlock = BlockPickup(drive, score1stBlock.end(), arm, 1);
   TrajectorySequence score2ndBlock = BlockScore(drive, driveTo2ndBlock.end(), arm);

   TrajectorySequence driveTo3rdBlock = BlockPickup(drive, score2ndBlock.end(), arm, 2);
   TrajectorySequence score3rdBlock = BlockScore(drive, driveTo3rdBlock.end(), arm);

   TrajectorySequence park = Parking(drive, score3rdBlock.end());

   drive.followTrajectorySequence(BlueOP);
    arm.setShoulderPosition(0.4);
   arm.setElbowPosition(0.125);
   arm.setLiftPosition(LiftClass.liftPosition.SUBMERSIBLE);
   sleep(1000);
   arm.setPulleyPosition(LiftClass.PulleyPosition.SUBMERSIBLE);
   sleep(1000);
   arm.setShoulderPosition(0.25);
   arm.setElbowPosition(0);



   sleep(500);
   arm.setShoulderPosition(0.675);
   arm.setElbowPosition(0);

   sleep(500);
   arm.setLiftPosition(LiftClass.liftPosition.HOME);


   sleep(700);
   arm.setClawPosition(1);

   sleep(500);
   arm.setShoulderPosition(0.25);
   arm.setElbowPosition(0);



   sleep(1000);
   arm.setPulleyPosition(LiftClass.PulleyPosition.HOME);






   sleep(400);

   drive.followTrajectorySequence(BlueOG);

   /*drive.followTrajectorySequence(InitialBlock);
   drive.followTrajectorySequence(driveTo1stBlock);
   drive.followTrajectorySequence(score1stBlock);
   drive.followTrajectorySequence(driveTo2ndBlock);
   drive.followTrajectorySequence(score2ndBlock);
   drive.followTrajectorySequence(driveTo3rdBlock);
   drive.followTrajectorySequence(score3rdBlock);
   drive.followTrajectorySequence(park);*/

   poseStorage.currentPose = drive.getPoseEstimate();
  }
 }