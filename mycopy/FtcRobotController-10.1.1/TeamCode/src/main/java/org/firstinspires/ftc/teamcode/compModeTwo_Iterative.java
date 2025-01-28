package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.external_methods.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.TouchSensor;
//import org.opencv.core.
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "compModeTwo_iterative", group = "Iterative OpMode")
public class compModeTwo_Iterative extends OpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx pivotOne = null;
    private DcMotorEx pivotTwo = null;
    private DcMotorEx linSlideLeft = null;
    private DcMotorEx linSlideRight = null;
    private Servo intakeServo = null;
    private TouchSensor slideLimit = null;
    // private IMU imu = null;
    private int ZERO= 0, LOW_RUNG= 400, HIGH_RUNG= 615,LOW_BASET = 640, GROUND = 40, SUB = 150;
    private int pivotPose = 0;
    //private SparkFunOTOS imu;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        /*imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);*/

        //imu = hardwareMap.get(SparkFunOTOS.class, "imu");
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");
        pivotOne = hardwareMap.get(DcMotorEx.class, "pivotOne");
        pivotTwo = hardwareMap.get(DcMotorEx.class, "pivotTwo");
        linSlideLeft = hardwareMap.get(DcMotorEx.class, "linSlideLeft");
        linSlideRight = hardwareMap.get(DcMotorEx.class, "linSlideRight");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        slideLimit = hardwareMap.get(TouchSensor.class, "slideTouchLimit");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        linSlideRight.setDirection(DcMotorSimple.Direction.FORWARD);
        linSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        pivotOne.setDirection(DcMotorSimple.Direction.FORWARD);
        pivotTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeServo.setDirection(Servo.Direction.FORWARD);

        reset_runWithEncoder(pivotOne, pivotTwo);
        reset_runWithoutEncoder(linSlideLeft,linSlideRight);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
        //resetYaw();
    }
    @Override
    public void loop() {
        boolean lowRungButton = gamepad1.dpad_up;
        boolean highRungButton = gamepad1.dpad_right;
        boolean submersibleButton = gamepad1.dpad_left;
        boolean slideOutTrigger = gamepad1.right_bumper;
        boolean slideInTrigger = gamepad1.left_bumper;
        boolean intakeCloseButton = gamepad1.triangle;
        boolean pivotZeroButton = gamepad1.circle;
        boolean groundButton = gamepad1.dpad_down;

        drivetrain(-gamepad1.left_stick_y, gamepad1.left_stick_x*1.1,-gamepad1.right_stick_x, leftFrontDrive, leftBackDrive,rightFrontDrive,rightBackDrive);

        if (lowRungButton) {
           // pivotPose = LOW_RUNG;
            pivotRun(LOW_RUNG, pivotOne, pivotTwo);
        }else if(highRungButton){
           // pivotPose = HIGH_RUNG;
            pivotRun(HIGH_RUNG, pivotOne, pivotTwo);
        }else if(submersibleButton) {
           // pivotPose = SUB;
            pivotRun(SUB, pivotOne, pivotTwo);
        }else if(pivotZeroButton){
           // pivotPose = ZERO;
            pivotRun(ZERO, pivotOne, pivotTwo);
        }else if(groundButton){
           // pivotPose = GROUND;
            pivotRun(GROUND, pivotOne, pivotTwo);
        }

        if(slideOutTrigger){
            slide(linSlideLeft.getCurrentPosition(), -0.8);
        }else if(slideInTrigger && !slideLimit.isPressed()){
            slide(linSlideLeft.getCurrentPosition(), 0.8);
        }


        if(intakeCloseButton){
            intakeClose(intakeServo);
        }else{
            intakeOpen(intakeServo);
        }

        if(gamepad1.touchpad){
            reset_runWithoutEncoder(linSlideLeft, linSlideRight);
        }else if(gamepad1.share){
            reset_runWithEncoder(pivotOne, pivotTwo);
        }

        //pivotRun(pivotPose, pivotOne, pivotTwo);

        telemetry.addData("Pivot Encoder Pos", "%s, %s", pivotOne.getCurrentPosition(), pivotTwo.getCurrentPosition());
        telemetry.addData("Intake Pos", "%s",intakeServo.getPosition());
        telemetry.addData("Lin Slide Encoder L | R", "%s, %s", linSlideLeft.getCurrentPosition(), linSlideRight.getCurrentPosition());
        telemetry.update();
    }

    private void slide(int current_position, double power){
        if(current_position >= -5000 && current_position < 1) {
            linSlideLeft.setPower(power);
            linSlideRight.setPower(power);
        }else{
            linSlideLeft.setPower(0);
            linSlideRight.setPower(0);
        }
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
