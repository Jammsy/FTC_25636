package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class drive_and_reset_Auto extends OpMode{
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
    private IMU imu = null;
    private double slideMax = -5000;
    private int ZERO= 0, HIGH_RUNG= 600;//LOW_RUNG= 400, HIGH_RUNG= 600,LOW_BASET = 640, GROUND = 55, SUB = 150;
    private enum pivotStates {START, RAISE, SLIDE, INTAKE_CLOSE, INTAKE_OPEN, DRIVE};
    private pivotStates pivotState = pivotStates.START;
    private int pivotPose = 0;



    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

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

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivotOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        resetSlideEncoders();
        runtime.reset();
        resetYaw();
    }

    @Override
    public void loop() {
        linSlideLeft.setPower(0.8);
        linSlideRight.setPower(0.8);
        drivetrain(leftFrontDrive.getCurrentPosition());
        pivotRun(pivotPose);
        telemetry.addData("Left Front Drive encoder pos", "%s %s", leftFrontDrive.getCurrentPosition() );
        telemetry.addData("Left Back Drive Encoder pos", "%s %s", leftBackDrive.getCurrentPosition());
        telemetry.addData("Right Front Drive encoder pos", "%s %s", rightFrontDrive.getCurrentPosition());
        telemetry.addData("Right Back Drive Encoder Pos" , "%s %s", rightBackDrive.getCurrentPosition());

        switch(pivotState){

            case START :
                intakeClose();
                pivotState = pivotStates.RAISE;

                break;
            case DRIVE:



            case RAISE :
                pivotPose = HIGH_RUNG;
                linSlideLeft.setTargetPosition(-3500);
                linSlideRight.setTargetPosition(-3500);
                pivotState = pivotStates.SLIDE;
                break;
            case SLIDE :
                linSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }

        pivotRun(pivotPose);
        telemetry.addData("Pivot Encoder Pos", "%s, %s", pivotOne.getCurrentPosition(), pivotTwo.getCurrentPosition());
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
    public void drivetrain(int current_pos){

    }

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

    public void resetSlideEncoders() {
        linSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Usually you want to use encoders
        linSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Unless you are using a limit switch
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public void intakeOpen() {
        intakeServo.setPosition(0.35);
    }

    public void intakeClose() {
        intakeServo.setPosition(0.65);
    }
    public void pivotRun(int e){
        if(e != 0) {
            pivotOne.setPower(0.6);
            pivotTwo.setPower(0.6);
            pivotOne.setTargetPosition(e);
            pivotTwo.setTargetPosition(e);
            pivotOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivotTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else{
            pivotOne.setPower(0);
            pivotTwo.setPower(0);
            pivotOne.setTargetPosition(e);
            pivotTwo.setTargetPosition(e);
        }
    }

    public void slideOut(int pos, double max) {
        if (pos < max) {
            linSlideLeft.setPower(0.6);
            linSlideRight.setPower(0.6);
        } else {
            linSlideLeft.setPower(0);
            linSlideRight.setPower(0);
        }
    }

    public void slideIn(int slidePos) {
        if (slidePos > 0) {
            linSlideLeft.setPower(-0.6); // Made negative to retract
            linSlideRight.setPower(-0.6);
        } else {
            linSlideLeft.setPower(0);
            linSlideRight.setPower(0);
        }
    }
}
