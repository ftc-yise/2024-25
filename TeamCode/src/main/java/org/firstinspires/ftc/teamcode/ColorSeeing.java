package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.yise.OpenCVVision;
@TeleOp(name="OpenCVTest", group="Vision")
public class ColorSeeing extends LinearOpMode {

    OpenCVVision vision = new OpenCVVision();


    // Declare OpMode members for each of the 4 motors.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private Servo outtake = null;

    private boolean loop = true;

    @Override
    public void runOpMode() {
        // create instance of OpenCV class
        OpenCVVision vision = new OpenCVVision(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");

        outtake = hardwareMap.get(Servo.class, "outtake");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        vision.setCameraPipeline(OpenCVVision.Color.RED);// Wait for the game to start (driver presses PLAY)

        telemetry.addData("Status", "Initialized");

        telemetry.addData("Color:", vision.getColor());
        telemetry.addData("centroid", vision.getLargestContourCentroid());

        telemetry.addLine();

        telemetry.addData("Corner T", vision.getTopRightCorner());
        telemetry.addData("Corner B", vision.getBottomRightCorner());
        telemetry.update();

            if (gamepad1.right_trigger > 0.75){
                vision.setCameraPipeline(OpenCVVision.Color.BLUE);
            } else if (gamepad1.left_trigger > 0.75){
                vision.setCameraPipeline(OpenCVVision.Color.RED);
            } else {
                vision.setCameraPipeline(OpenCVVision.Color.YELLOW);
            }

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.right_trigger > 0.75){
                vision.setCameraPipeline(OpenCVVision.Color.BLUE);
            } else if (gamepad1.left_trigger > 0.75){
                vision.setCameraPipeline(OpenCVVision.Color.RED);
            } else {
                vision.setCameraPipeline(OpenCVVision.Color.YELLOW);
            }

            telemetry.addData("Color:", vision.getColor());
            telemetry.addData("centroid", vision.getLargestContourCentroid());

            telemetry.addLine();

            telemetry.addData("Corner T", vision.getTopRightCorner());
            telemetry.addData("Corner B", vision.getBottomRightCorner());
            telemetry.update();

        }
    }
}