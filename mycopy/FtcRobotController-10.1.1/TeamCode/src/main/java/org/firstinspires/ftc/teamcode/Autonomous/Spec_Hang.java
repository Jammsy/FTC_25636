package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.*;

import static org.firstinspires.ftc.teamcode.Constants.Constants.PivotConstants.*;
@Autonomous
public class Spec_Hang extends LinearOpMode {
    Drivetrain m_drive = null;
    Intake m_intake = null;
    Slide m_slide = null;
    Pivot m_pivot = null;


    @Override
    public void runOpMode() throws InterruptedException {
        m_drive = new Drivetrain(hardwareMap);
        m_intake = new Intake(hardwareMap);
        m_slide = new Slide(hardwareMap);
        m_pivot = new Pivot(hardwareMap);

        m_pivot.resetEncoders();
        m_slide.resetEncoders();

        waitForStart(); // Wait for the start button to be pressed

        m_intake.closeIntake();
        m_pivot.pivotRun(HIGH_RUNG);
        sleep(1000);

        m_slide.slideOut(-0.8);
        sleep(2000);

        m_drive.setMotorPower(0.545, 0.5, 0.5, 0.5);
        sleep(2000);

        m_slide.slideIn(1);
        sleep(2000);

        m_intake.openIntake();
        m_drive.setMotorPower(-1.045, -1, -1, -1);
        sleep(1000);

        m_pivot.pivotRun(GROUND);
        m_drive.setMotorPower(1.045, 1, -1, -1);
        sleep(3000);

        m_drive.setMotorPower(1.045, 1, 1, 1);
        sleep(1000);

        m_drive.setMotorPower(0.745, 0.7, -0.7, -0.7);
        sleep(1000);

        m_pivot.pivotRun(HIGH_RUNG);
        m_drive.setMotorPower(0.445, 0.4, 0.4, 0.4);
        sleep(1000);

        m_pivot.setPower(-0.1);
        m_pivot.stopPivot();
        sleep(1000);

        m_drive.stopDriving(); // Stop all motors at the end
        m_slide.stopSlide(); // Stop the slide motor
    }
}
