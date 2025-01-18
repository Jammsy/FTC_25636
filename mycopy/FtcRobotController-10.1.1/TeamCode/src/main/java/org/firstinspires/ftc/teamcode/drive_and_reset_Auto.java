package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.external_methods.*;
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
//turt
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
    private double slideMax = -5000;
    private int ZERO= 0, HIGH_RUNG= 600;//LOW_RUNG= 400, HIGH_RUNG= 600,LOW_BASET = 640, GROUND = 55, SUB = 150;
    private enum pivotStates {START, RAISE, SLIDE, INTAKE_CLOSE, INTAKE_OPEN, DRIVE};
    private pivotStates pivotState = pivotStates.START;
    private int pivotPose = 0;



    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

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
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pivotOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        linSlideLeft.setPower(0.8);
        linSlideRight.setPower(0.8);
        telemetry.addData("Left Front Drive encoder pos", "%s %s", leftFrontDrive.getCurrentPosition() );
        telemetry.addData("Left Back Drive Encoder pos", "%s %s", leftBackDrive.getCurrentPosition());
        telemetry.addData("Right Front Drive encoder pos", "%s %s", rightFrontDrive.getCurrentPosition());
        telemetry.addData("Right Back Drive Encoder Pos" , "%s %s", rightBackDrive.getCurrentPosition());

        switch(pivotState){

            case START :
                intakeClose(intakeServo);
                pivotState = pivotStates.RAISE;
                break;
            case RAISE :
                pivotPose = HIGH_RUNG;
                pivotRun(pivotPose, pivotOne, pivotTwo);
                linSlideLeft.setTargetPosition(-3500);
                linSlideRight.setTargetPosition(-3500);
                pivotState = pivotStates.SLIDE;
                break;

            case DRIVE:
               drivetrain(5000, 0.5, leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);
               pivotState = pivotStates.SLIDE;
               break;

            case SLIDE :
                runtime.reset();
                linSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(runtime.seconds() >= 3){
                    drivetrain(0, -0.7, leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);
                }
        }

        telemetry.addData("Pivot Encoder Pos", "%s, %s", pivotOne.getCurrentPosition(), pivotTwo.getCurrentPosition());
        telemetry.addData("Lin Slide Encoder L | R", "%s, %s", linSlideLeft.getCurrentPosition(), linSlideRight.getCurrentPosition());
        telemetry.update();
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


}
