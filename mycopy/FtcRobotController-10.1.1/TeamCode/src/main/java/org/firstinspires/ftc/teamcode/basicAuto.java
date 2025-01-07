/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
//@Disabled
public class    basicAuto extends OpMode
{
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
    private Servo intakePivot = null;
    private IMU imu = null;
    private double intakeForwardPosition = 0.15;
    private double intakeBackwardPosition = 1;
    private double pivotPos = 100000;
    private int dtForwardPos = 0;
    private int dtBackPos = 0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        //IMU variable / hardware map
        imu = hardwareMap.get(IMU.class, "imu");

        //IMU parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        //Hardware map
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        pivotOne = hardwareMap.get(DcMotor.class, "pivotOne");
        pivotTwo = hardwareMap.get(DcMotor.class, "pivotTwo");
        linSlideLeft = hardwareMap.get(DcMotor.class, "linSlideLeft");
        linSlideRight = hardwareMap.get(DcMotor.class, "linSlideRight");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        //Motor Direction Set
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        pivotOne.setDirection(DcMotor.Direction.REVERSE);
        pivotTwo.setDirection(DcMotor.Direction.FORWARD);
        linSlideRight.setDirection(DcMotorSimple.Direction.FORWARD);
        linSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakePivot.setDirection(Servo.Direction.REVERSE);

        //Breaking modes for motors
        /*leftFrontDrive.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        leftBackDrive.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        rightFrontDrive.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        rightBackDrive.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        rightFrontDrive.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));*/
        pivotOne.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        pivotTwo.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        linSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pivotOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.resetYaw();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        intakeClose();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

    }
    public void driveTrain(int pos){
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setTargetPosition(pos);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setTargetPosition(pos);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setTargetPosition(pos);

        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setTargetPosition(pos);
    }

    public void resetSlideEncoders(){
        linSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetYaw(){
        imu.resetYaw();
    }
    public void pivotUp(){
            pivotOne.setPower(0.65);
            pivotTwo.setPower(0.65);
    }
    public void pivotDown(){
        pivotOne.setPower(-0.45);
        pivotTwo.setPower(-0.45);
    }
    public void pivotStop(){
        pivotOne.setPower(0);
        pivotTwo.setPower(0);
    }
    public void intakeOpen(){
        intakePivot.setPosition(0.75);
    }
    public void intakeClose(){
        intakeServo.setPosition(0.65);
    }
    public void intakeForward(){
        intakePivot.setPosition(intakeForwardPosition);
    }
    public void intakeStow(){
        intakePivot.setPosition(intakeBackwardPosition);

    }
    public void slideOut(){
        linSlideLeft.setPower(0.6);
        linSlideRight.setPower(0.6);
    }

    @Override
    public void stop() {
        pivotStop();
    }
}
