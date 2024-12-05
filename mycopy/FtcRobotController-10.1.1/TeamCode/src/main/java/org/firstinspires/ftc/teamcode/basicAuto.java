package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "basicAuto")
//@Disabled
public class basicAuto extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor pivotOne = null;
    private DcMotor pivotTwo = null;
    private Servo intakeServo = null;

    @Override
    public void runOpMode() {
        //Hardware map
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        pivotOne = hardwareMap.get(DcMotor.class, "pivotOne");
        pivotTwo = hardwareMap.get(DcMotor.class, "pivotTwo");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        //Motor Direction Set
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        pivotOne.setDirection(DcMotor.Direction.REVERSE);
        pivotTwo.setDirection(DcMotor.Direction.FORWARD);
        //Breaking modes for motors
        /*leftFrontDrive.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        leftBackDrive.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        rightFrontDrive.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        rightBackDrive.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        rightFrontDrive.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));*/
        pivotOne.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        pivotTwo.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Encoders Reset Debug FL/FR: ", "%4.2f, %4.2f", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
        telemetry.addData("                     Bl/BR: ", "%4.2f, %4.2f", leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
        telemetry.update();

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            int forward = 1000;
            int backward = -1000;
            int armPos = 0;

            //Arm Pivot Encoder Stuff
            /*
            double CPR1 = 336;
            double pivot1Encoder = pivotOne.getCurrentPosition();
            double pivot2Encoder = pivotTwo.getCurrentPosition();

            double p1Rev = pivot1Encoder / CPR1;
            double p2Rev = pivot2Encoder / CPR1;

            double p1Angle = p1Rev * 360;
            double p1Ang = p1Angle % 360;
            double p2Angle = p2Rev * 360;
            double p2Ang = p2Angle % 360;
*/
            //Drivetrain Encoder stuff
            /*
            double CPR = 252;
            double frEncoder = rightFrontDrive.getCurrentPosition();
            double flEncoder = leftFrontDrive.getCurrentPosition();
            double brEncoder = rightBackDrive.getCurrentPosition();
            double blEncoder = leftBackDrive.getCurrentPosition();

            double frRev = frEncoder/CPR;
            double flRev = flEncoder/CPR;
            double brRev = brEncoder/CPR;
            double blRev = blEncoder/CPR;

            double angle1 = frRev * 360;
            double frAng = angle1 % 360;
            double angle2 = flRev * 360;
            double flAng = angle2 % 360;
            double angle3 = brRev * 360;
            double brAng = angle3 % 360;
            double angle4 = blRev * 360;
            double blAng = angle4 % 360;
*/

            rightFrontDrive.setTargetPosition(forward);
            leftFrontDrive.setTargetPosition(forward);
            rightBackDrive.setTargetPosition(-(forward));
            rightBackDrive.setTargetPosition(-(forward));
            pivotOne.setTargetPosition(armPos);
            pivotTwo.setTargetPosition(armPos);

            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivotOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivotTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightFrontDrive.setPower(0.5);
            leftFrontDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
            pivotOne.setPower(0.6);
            pivotTwo.setPower(0.6);
            //intakeServo.setPosition(0.75);

            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //Show status, DT wheel power, and slide / intake debug
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Intake Position", "%2f", intakeServo.getPosition());
            telemetry.addData("Arm Position Motor1/Motor2", "%4.3f, %4.3f", pivotOne.getCurrentPosition(), pivotTwo.getCurrentPosition());
            telemetry.addData("DT Power Levels FL/FR", "%4.2f, %4.2F", leftFrontDrive.getPower(), rightFrontDrive.getPower());
            telemetry.addData("DT Power Levels BL/BR","%4.2f, %4.2f", leftBackDrive.getPower(),rightBackDrive.getPower());
            //telemetry.addData("Encoder Pos Front : Left/Right", "%4.3f, %4.3f", flEncoder, frEncoder);
            //telemetry.addData("Encoder Pos Back : Left/Right", "%4.3f, %4.3f", blEncoder, brEncoder);
            telemetry.update();

        }
    }
}

