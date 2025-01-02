package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Basic: compModeOne_linear", group="Linear OpMode")
//@Disabled
public class compModeOne_linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor pivotOne = null;
    private DcMotor pivotTwo = null;
    private DcMotor linSlideLeft = null;
    private DcMotor linSlideRight = null;
    private Servo intakeServo = null;
    private Servo intakePivot = null;
    private IMU imu = null;
    private double intakeForwardPosition = 0.15;
    private double intakeBackwardPosition = 1;
    private double slideLCurrentPos = 0.0;
    private double slideRCurrentPos = 0.0;

    @Override
    public void runOpMode() {

        //IMU variable / hardware map
        imu = hardwareMap.get(IMU.class, "imu");

        //IMU parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        //Hardware map
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        pivotOne = hardwareMap.get(DcMotor.class, "pivotOne");
        pivotTwo = hardwareMap.get(DcMotor.class, "pivotTwo");
        linSlideLeft = hardwareMap.get(DcMotor.class, "linSlideLeft");
        linSlideRight = hardwareMap.get(DcMotor.class, "linSlideRight");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        //Motor Direction Set
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        pivotOne.setDirection(DcMotor.Direction.REVERSE);
        pivotTwo.setDirection(DcMotor.Direction.FORWARD);
        linSlideRight.setDirection(DcMotorSimple.Direction.FORWARD);
        linSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakePivot.setDirection(Servo.Direction.REVERSE);

        //Breaking modes for motors
        /*leftFrontDrive.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        leftBackDrive.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        rightFrontDrive.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        rightBackDrive.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        rightFrontDrive.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));*/
        pivotOne.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        pivotTwo.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        linSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideLCurrentPos = linSlideLeft.getCurrentPosition();
        slideRCurrentPos = linSlideLeft.getCurrentPosition();

        imu.resetYaw();

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //double max;
            double pivot;
            double slide;
            slideLCurrentPos = linSlideLeft.getCurrentPosition();
            slideRCurrentPos = linSlideRight.getCurrentPosition();
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

//Robot Centric Control within comment bracket.
            /*// POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  -gamepad1.left_stick_x;
            double yaw     = -gamepad1.right_stick_x;//gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;


            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));


            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
*/

            //IMU direction reset button
            if (gamepad1.options) {
                imu.resetYaw();
            }
            if(gamepad1.dpad_left){
                linSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                linSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                linSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                linSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            //pivot logic
            if(gamepad1.left_trigger > 0.2){
                pivot = gamepad1.left_trigger;
            }else if(gamepad1.right_trigger > 0.2) {
                pivot = -(gamepad1.right_trigger);
            }else{
                pivot = 0;
            }

            //linear og slide logic
            if(gamepad1.dpad_up && (double)(linSlideRight.getCurrentPosition()) < 3000 && (double)(linSlideLeft.getCurrentPosition()) < 3000){
                slide = 0.6;
            }else if(gamepad1.dpad_down && (double)(linSlideRight.getCurrentPosition()) > 1){
                slide = -0.6;
            }else{
                slide = 0;
            }


            //intake logic
            if(gamepad1.y){
                intakeServo.setPosition(0.75);
            }else{
                intakeServo.setPosition(0.65);
            }

            if(gamepad1.left_bumper){
                intakePivot.setPosition(intakeForwardPosition);
            } else if (gamepad1.right_bumper){
                intakePivot.setPosition(intakeBackwardPosition);
            }

            //Set Power To motors as well as reset defaults
            pivotOne.setPower(pivot);
            pivotTwo.setPower(pivot);
            linSlideRight.setPower(slide);
            linSlideLeft.setPower(slide);


            //Show status, DT wheel power, and slide / intake debug
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("Slide Pos R/L", "%2f , %2f", (double)(linSlideRight.getCurrentPosition()), (double)(linSlideLeft.getCurrentPosition()));
            telemetry.addData("Intake Pivot Position", "%s", intakePivot.getPosition());
            telemetry.addData("Heading", "%s", botHeading);
            telemetry.addData("Pivot Stall Power", "%s, %s", pivotOne.getPower(), pivotTwo.getPower());
            telemetry.update();

            //Rumble controller after 2 Minutes
           /* if(runtime.seconds() > 120){
                gamepad1.rumble(300);
                gamepad1.stopRumble();
            }*/
        }
    }
}

