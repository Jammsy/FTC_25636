package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.localization.constants.OTOSConstants;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.Constants;

public class Drivetrain {
    private DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public Drivetrain(HardwareMap hardwareMap){
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightDrive");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void stopDriving(){
        setMotorPower(0,0,0,0);
    }

    public void setMotorPower(double FL, double FR, double BL, double BR){
        frontLeftMotor.setPower(FL);
        frontRightMotor.setPower(FR);
        backLeftMotor.setPower(BL);
        backRightMotor.setPower(BR);
    }

    public void drive_Cartesian(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double FLP = (y + x + rx) / denominator;
        double BLP = (y - x + rx) / denominator;
        double FRP = (y - x - rx) / denominator;
        double BRP = (y + x - rx) / denominator;

        setMotorPower(FLP, FRP, BLP, BRP);
    }
}




