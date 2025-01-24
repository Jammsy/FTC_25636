package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class external_methods  extends compModeTwo_Iterative{
    public static void drivetrain(double y,double x, double rx, DcMotor one, DcMotor two,DcMotor three, DcMotor four){
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        one.setPower(frontLeftPower);
        two.setPower(backLeftPower);
        three.setPower(frontRightPower);
        four.setPower(backRightPower);
    }
    //turt
    public static void drivetrain(double y,double x, double rx, double heading, DcMotor one, DcMotor two,DcMotor three, DcMotor four){
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        one.setPower(frontLeftPower);
        two.setPower(backLeftPower);
        three.setPower(frontRightPower);
        four.setPower(backRightPower);
    }

    public static void drivetrain(int pos, double power, DcMotor one, DcMotor two,DcMotor three, DcMotor four){
        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        one.setPower(power);
        one.setTargetPosition(pos);
        one.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        do {
            two.setPower(power+.2);
            three.setPower(power);
            four.setPower(power+.1);
        }while(one.isBusy());

    }

    public static void reset_runWithoutEncoder(DcMotor one, DcMotor two) {
        one.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        two.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        one.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Usually you want to use encoders
        two.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Unless you are using a limit switch
    }
    public static void reset_runWithEncoder(DcMotor one, DcMotor two){
        one.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        two.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void intakeOpen(Servo s) {
        //0.65
        s.setPosition(0.5);

    }

    public static void intakeClose(Servo s) {
        //0.75
        s.setPosition(0.8);
    }
    public static void pivotRun(int e, DcMotor one, DcMotor two){
        if(e != 0) {
            one.setPower(0.6);
            two.setPower(0.6);
            one.setTargetPosition(e);
            two.setTargetPosition(e);
            one.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            two.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else{
            one.setPower(0);
            two.setPower(0);
            one.setTargetPosition(e);
            two.setTargetPosition(e);
        }
    }
    public static void slideOut(int pos, double max, DcMotor one, DcMotor two) {
        if (pos < max) {
            one.setPower(0.6);
            two.setPower(0.6);
        } else {
            one.setPower(0);
            two.setPower(0);
        }
    }

    public static void slideIn(int slidePos, DcMotor one, DcMotor two) {
        if (slidePos < 0) {
            one.setPower(-0.6); // Made negative to retract
            two.setPower(-0.6);
        } else {
            one.setPower(0);
            two.setPower(0);
        }
    }

    public static void configureIMU(SparkFunOTOS x){
        x.setLinearUnit(DistanceUnit.INCH);
        x.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        x.setOffset(offset);
    }
}
