package org.firstinspires.ftc.teamcode.Subsystems;

//import com.pedropathing.localization.constants.OTOSConstants;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import static org.firstinspires.ftc.teamcode.utils.*;

public class Drivetrain {
    private IMU imu = null;
    private DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    public Drivetrain(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new Rev9AxisImuOrientationOnRobot(
                Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP, Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.LEFT));
        imu.initialize(parameters);


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

    public void stopDriving() {
        setMotorPower(0, 0, 0, 0);
    }

    public void setMotorPower(double FL, double FR, double BL, double BR) {
        frontLeftMotor.setPower(FL);
        frontRightMotor.setPower(FR);
        backLeftMotor.setPower(BL);
        backRightMotor.setPower(BR);
    }

    public void drive_Cartesian(double x, double y, double rx) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        rx = rx * 1.1;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double FLP = (rotY + rotX + rx) / denominator;
        double BLP = (rotY - rotX + rx) / denominator;
        double FRP = (rotY - rotX - rx) / denominator;
        double BRP = (rotY + rotX - rx) / denominator;

        setMotorPower(FLP, FRP, BLP, BRP);
    }

    public void encoderDrive(int position, double power){
        reset_runWithEncoder(frontLeftMotor);
        frontLeftMotor.setTargetPosition(position);
        frontLeftMotor.setPower(power);
        if(frontLeftMotor.getPowerFloat() && !(frontLeftMotor.getCurrentPosition()==position)){
            frontRightMotor.setPower(power);
            backLeftMotor.setPower(power);
            backRightMotor.setPower(power);
        }else {
            stopDriving();
        }
        setRunWithoutEncoder(frontLeftMotor);
    }
}