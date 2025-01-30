package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.localization.constants.OTOSConstants;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import static org.firstinspires.ftc.teamcode.utils.*;

public class Drivetrain {
    private DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public Drivetrain(HardwareMap hardwareMap){
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "rightBackDrive");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setRunWithoutEncoder(frontLeftMotor, frontRightMotor);
        setRunWithoutEncoder(backLeftMotor, backRightMotor);
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




