package org.firstinspires.ftc.teamcode.Autonomous;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.UTILITIES.utils.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class specimine_parkR extends OpMode{
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
    private int ZERO= 0, HIGH_RUNG= 600;
    private enum pivotStates {START, RAISE, SLIDE,DRIVE,BACKUP,MRIGHT, END};
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

        linSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        reset_runWithEncoder(pivotOne, pivotTwo);
        reset_runWithoutEncoder(linSlideLeft, linSlideRight);
        reset_runWithoutEncoder(leftBackDrive,leftFrontDrive);
        reset_runWithoutEncoder(rightBackDrive,rightFrontDrive);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        intakeServo.setPosition(0.85);
        pivotTwo.setPower(0.65);
        pivotOne.setPower(0.65);
    }

    @Override
    public void loop() {
        intakeServo.setPosition(0.85);
        switch(pivotState){

            case START :
                pivotState = pivotStates.RAISE;
                break;

            case RAISE :
                pivotPose = HIGH_RUNG;
                pivotRun(pivotPose);
                pivotState = pivotStates.DRIVE;
                break;

            case DRIVE:
                slide(2, -0.7, linSlideRight, linSlideLeft);
                drivetrain(1, 0.42);
                pivotState = pivotStates.SLIDE;
                break;

            case SLIDE :
                slide(2,0.7, linSlideRight, linSlideLeft);
                pivotRun(1);
                drivetrain(1, -0.5);
                pivotState = pivotStates.MRIGHT;
                break;

            case MRIGHT:
                strafe(2, 0.6);
                pivotState = pivotStates.END;
                break;

            default:
                intakeServo.setPosition(0.85);
                linSlideLeft.setPower(0);
                linSlideRight.setPower(0);
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                pivotOne.setPower(0);
                pivotTwo.setPower(0);
                break;
        }

        telemetry.addData("Pivot Encoder Pos", "%s, %s", pivotOne.getCurrentPosition(), pivotTwo.getCurrentPosition());
        telemetry.addData("Lin Slide Encoder L | R", "%s, %s", linSlideLeft.getCurrentPosition(), linSlideRight.getCurrentPosition());
        telemetry.update();
    }
    private void strafe(int time, double power) {
        ElapsedTime runtime = new ElapsedTime();
        while(runtime.seconds() < time){
            leftFrontDrive.setPower((power + 0.045));
            leftBackDrive.setPower(-(power));
            rightFrontDrive.setPower(-(power));
            rightBackDrive.setPower(power);
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void turn(int time, double power){
        ElapsedTime runtime = new ElapsedTime();
        while(runtime.seconds() < time){
            leftFrontDrive.setPower(-(power + 0.045));
            leftBackDrive.setPower(-(power));
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    private void drivetrain(double time, double power){
        ElapsedTime timer = new ElapsedTime();
        while(timer.seconds() <= time){
            if(power <0) {
                leftFrontDrive.setPower((power - 0.045));
            }else {
                leftFrontDrive.setPower((power + 0.04));
            }
            leftBackDrive.setPower(power);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    private void slide(double time, double power, DcMotor one, DcMotor two){
        ElapsedTime times = new ElapsedTime();
        while(times.seconds() <= time){
            one.setPower(power);
            two.setPower(power);
        }
        one.setPower(0);
        two.setPower(0);
    }
    private void pivotRun(int pos){
        pivotTwo.setTargetPosition(pos);
        pivotOne.setTargetPosition(pos);
        if(pos != 0) {
            pivotOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            pivotTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }else{
            pivotTwo.setPower(0);
            pivotOne.setPower(0);
        }

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
