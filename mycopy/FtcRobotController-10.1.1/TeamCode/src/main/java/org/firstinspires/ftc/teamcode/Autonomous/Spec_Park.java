package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import static org.firstinspires.ftc.teamcode.Constants.Constants.PivotConstants.*;

@Autonomous
public class Spec_Park extends LinearOpMode {

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

        waitForStart();
        if (opModeIsActive()) {
            m_intake.closeIntake();
            sleep(250);

            m_pivot.pivotRun(HIGH_RUNG);
            sleep(500);

            m_slide.setPower(0.8);
            sleep(2000);

            m_slide.stopSlide();
            m_drive.setMotorPower(0.545, 0.5, 0.5, 0.5);
            sleep(1550);

            m_drive.setMotorPower(0, 0, 0, 0);
            m_slide.setPower(-0.8);
            sleep(2000);

            m_intake.openIntake();
            m_drive.setMotorPower(-1.045, -1, -1, -1);
            sleep(550);

            m_drive.setMotorPower(0, 0, 0, 0);
            m_pivot.pivotRun(GROUND);
            m_drive.setMotorPower(0.745, -0.7, -0.7, 0.7);
            sleep(2000);

            m_drive.stopDriving();
            m_pivot.stopPivot();
            m_slide.stopSlide();
        }
    }
}