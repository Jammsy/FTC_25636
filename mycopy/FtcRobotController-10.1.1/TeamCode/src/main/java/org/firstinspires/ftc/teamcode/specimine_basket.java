package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.external_methods.intakeClose;
import static org.firstinspires.ftc.teamcode.external_methods.intakeOpen;
import static org.firstinspires.ftc.teamcode.external_methods.pivotRun;
import static org.firstinspires.ftc.teamcode.external_methods.reset_runWithEncoder;
import static org.firstinspires.ftc.teamcode.external_methods.reset_runWithoutEncoder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class specimine_basket extends OpMode{
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
    private int ZERO= 0, HIGH_RUNG= 596,LOW_RUNG= 400,LOW_BASET = 640, GROUND = 55, SUB = 150;
    private enum pivotStates {START, RAISE, SLIDE,DRIVE,BACKUP,MLEFT,TLEFT, END};
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
        intakeClose(intakeServo);
        intakeServo.setPosition(0.85);
    }

    @Override
    public void loop() {
        intakeServo.setPosition(0.85);
        intakeClose(intakeServo);
        switch(pivotState){

            case START :
                intakeClose(intakeServo);
                pivotState = pivotStates.RAISE;
                break;

            case RAISE :
                pivotPose = HIGH_RUNG;
                pivotRun(pivotPose, pivotOne, pivotTwo);
                pivotState = pivotStates.DRIVE;
                break;

            case DRIVE:
                slide(2, -0.68);
                drivetrain(2, 0.27);
                pivotState = pivotStates.SLIDE;
                break;

            case SLIDE :
                slide(2,0.8);
                pivotRun(1, pivotOne, pivotTwo);
                drivetrain(1, -0.52);
                slide(2,-.7);
                pivotState = pivotStates.TLEFT;
                break;

            /*case MLEFT:
                strafe(2, 0.6);
                drivetrain(2, 0.4);
                pivotState = pivotStates.TLEFT;
                break;*/

            case TLEFT:
                turn(1, -0.45);
                pivotRun(LOW_BASET, pivotOne, pivotTwo);
                drivetrain(2, 0.27);
                slide(2,.65);
                intakeOpen(intakeServo);
                drivetrain(2,-.27);
                turn(1,.45);

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
        intakeClose(intakeServo);

        telemetry.addData("Left Front Drive encoder pos", "%s", leftFrontDrive.getCurrentPosition() );
        telemetry.addData("Left Back Drive Encoder pos", "%s", leftBackDrive.getCurrentPosition());
        telemetry.addData("Right Front Drive encoder pos", "%s", rightFrontDrive.getCurrentPosition());
        telemetry.addData("Right Back Drive Encoder Pos" , "%s", rightBackDrive.getCurrentPosition());
        telemetry.addData("Pivot Encoder Pos", "%s, %s", pivotOne.getCurrentPosition(), pivotTwo.getCurrentPosition());
        telemetry.addData("Lin Slide Encoder L | R", "%s, %s", linSlideLeft.getCurrentPosition(), linSlideRight.getCurrentPosition());
        telemetry.update();
    }
    private void strafe(int time, double power) {
        ElapsedTime runtime = new ElapsedTime();
        while(runtime.seconds() < time){
            if(power<0){
                leftFrontDrive.setPower(power - 0.045);
            }
            else{
                leftFrontDrive.setPower(power + 0.045);
            }
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
            if(power<0){
                leftFrontDrive.setPower(power - .045);
            }
            else{
                leftFrontDrive.setPower(-(power+0.045));
            }
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
    private void slide(double time, double power){
        ElapsedTime times = new ElapsedTime();
        while(times.seconds() <= time){
            linSlideRight.setPower(power);
            linSlideLeft.setPower(power);
        }
        linSlideRight.setPower(0);
        linSlideLeft.setPower(0);
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
