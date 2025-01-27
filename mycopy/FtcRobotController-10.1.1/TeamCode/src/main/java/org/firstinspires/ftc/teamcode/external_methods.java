package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class external_methods{
    public static void drivetrain(double y,double x, double rx, DcMotor one, DcMotor two,DcMotor three, DcMotor four){
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
    /*Field Oriented
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
    }*/
    public static void reset_runWithoutEncoder(DcMotor one) {
        one.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        one.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public static void reset_runWithEncoder(DcMotor one){
        one.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public static void setRunWithoutEncoder(DcMotor one){
        one.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public static void setRunWithEncoder(DcMotor one){
        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public static void setRunWithoutEncoder(DcMotor one, DcMotor two){
        one.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public static void setRunWithEncoder(DcMotor one, DcMotor two){
        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public static void reset_runWithoutEncoder(DcMotor one, DcMotor two) {
        one.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        two.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        one.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        s.setPosition(0.85);
    }
    public static void pivotRun(int e, DcMotor one, DcMotor two){
        if(e != 0) {
            one.setPower(0.7);
            two.setPower(0.7);
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
    /*public static void configureIMU(SparkFunOTOS x){
        x.setLinearUnit(DistanceUnit.INCH);
        x.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        x.setOffset(offset);
    }*/
}
