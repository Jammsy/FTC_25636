package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.external_methods.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "compModeTwo_iterative", group = "Iterative OpMode")
public class compModeTwo_Iterative extends OpMode{
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
   // private IMU imu = null;
    private double slideMax = -5000;
    private int ZERO= 0, LOW_RUNG= 400, HIGH_RUNG= 600,LOW_BASET = 640, GROUND = 54, SUB = 150;
    private int pivotPose = 0;
    //private SparkFunOTOS imu;
    //private double e = 0.0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        /*imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);*/

        //imu = hardwareMap.get(SparkFunOTOS.class, "imu");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        pivotOne = hardwareMap.get(DcMotor.class, "pivotOne");
        pivotTwo = hardwareMap.get(DcMotor.class, "pivotTwo");
        linSlideLeft = hardwareMap.get(DcMotor.class, "linSlideLeft");
        linSlideRight = hardwareMap.get(DcMotor.class, "linSlideRight");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        linSlideRight.setDirection(DcMotorSimple.Direction.FORWARD);
        linSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        pivotOne.setDirection(DcMotorSimple.Direction.FORWARD);
        pivotTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        pivotOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeServo.setDirection(Servo.Direction.FORWARD);

        pivotOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        reset_runWithoutEncoder(linSlideLeft, linSlideRight);
        reset_runWithEncoder(pivotOne, pivotTwo);
        runtime.reset();
        //resetYaw();
    }
    @Override
    public void loop() {
        boolean lowRungButton = gamepad1.dpad_up;
        boolean highRungButton = gamepad1.dpad_right;
        boolean submersibleButton = gamepad1.dpad_down;
        boolean slideOutTrigger = gamepad1.right_bumper;
        boolean slideInTrigger = gamepad1.left_bumper;
        boolean intakeCloseButton = gamepad1.triangle;
        boolean pivotZeroButton = gamepad1.circle;
        //boolean intakeOpenButton = gamepad1.cross;

        //double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        drivetrain(-gamepad1.left_stick_y, gamepad1.left_stick_x*1.1,-gamepad1.right_stick_x, leftFrontDrive, leftBackDrive,rightFrontDrive,rightBackDrive);
        //drivetrain(-gamepad1.left_stick_y, gamepad1.left_stick_x*1.1,-gamepad1.right_stick_x,bot_heading,leftFrontDrive, leftBackDrive,rightFrontDrive,rightBackDrive);

        if (lowRungButton) {
            pivotPose = LOW_RUNG;
        }else if(highRungButton){
            pivotPose = HIGH_RUNG;
        }else if(submersibleButton) {
            pivotPose = SUB;
        }else if(pivotZeroButton){
            pivotPose = ZERO;
        }else{
            pivotPose = GROUND;
        }

        if((linSlideLeft.getCurrentPosition() > -5000) && slideOutTrigger){
            linSlideLeft.setPower(-0.75);
            linSlideRight.setPower(-0.75);
            //slideOut(linSlideLeft.getCurrentPosition(), slideMax);
        }else if(slideInTrigger && (linSlideLeft.getCurrentPosition() <= -2)){
            linSlideLeft.setPower(0.75);
            linSlideRight.setPower(0.75);
            //slideIn(linSlideLeft.getCurrentPosition());
        }else {
            linSlideLeft.setPower(0);
            linSlideRight.setPower(0);
        }

        if(intakeCloseButton){
            //e+=0.01;
            intakeClose(intakeServo);
        }else{
            intakeOpen(intakeServo);
            //intakeClose(e);
        }

        if (gamepad1.options) {
            //resetYaw();
        }else if(gamepad1.touchpad){
            reset_runWithoutEncoder(linSlideLeft, linSlideRight);
        }else if(gamepad1.share){
            reset_runWithEncoder(pivotOne, pivotTwo);
        }

        pivotRun(pivotPose, pivotOne, pivotTwo);

        telemetry.addData("Pivot Encoder Pos", "%s, %s", pivotOne.getCurrentPosition(), pivotTwo.getCurrentPosition());
        telemetry.addData("Intake Pos", "%s",intakeServo.getPosition());
        telemetry.addData("Lin Slide Encoder L | R", "%s, %s", linSlideLeft.getCurrentPosition(), linSlideRight.getCurrentPosition());
        telemetry.update();
    }

    /*public void driveTrain(double y, double x, double rx, double botHeading) {
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
    }*/
    @Override
    public void stop() {
        //Good practice to stop motors in stop
        linSlideLeft.setPower(0);
        linSlideRight.setPower(0);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        pivotOne.setPower(0);
        pivotTwo.setPower(0);
    }
}
