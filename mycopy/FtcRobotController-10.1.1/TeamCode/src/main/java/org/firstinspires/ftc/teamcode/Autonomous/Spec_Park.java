package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import static org.firstinspires.ftc.teamcode.Constants.Constants.PivotConstants.*;

@Autonomous
public class Spec_Park extends OpMode {
    ElapsedTime runtime = new ElapsedTime();;
    Drivetrain m_drive = null;
    Intake m_intake = null;
    Slide m_slide = null;
    Pivot m_pivot = null;


    @Override
    public void init() {
        m_drive = new Drivetrain(hardwareMap);
        m_intake = new Intake(hardwareMap);
        m_slide = new Slide(hardwareMap);
        m_pivot = new Pivot(hardwareMap);

        m_pivot.resetEncoders();
        m_slide.resetEncoders();
    }

    @Override
    public void start(){
        runtime.reset();
        m_intake.closeIntake();
    }

    @Override
    public void loop() {
        while(runtime.seconds() <= 1){
            m_intake.closeIntake();
            m_pivot.pivotRun(HIGH_RUNG);
        }
        while(runtime.seconds() <= 3){
            m_slide.slideOut(-0.8);
        }
        while(runtime.seconds() <= 5){
            m_drive.setMotorPower(0.545, 0.5, 0.5, 0.5);
        }
        while(runtime.seconds() <= 7){
            m_slide.slideIn(1);
        }
        while(runtime.seconds() <= 8){
            m_intake.openIntake();
            m_drive.setMotorPower(-1.045, -1, -1, -1);
        }
        while(runtime.seconds() <= 11){
            m_pivot.pivotRun(GROUND);
            m_drive.setMotorPower(-1.045, -1, 1, 1);
        }
    }

    @Override
    public void stop(){
        m_drive.stopDriving();
        m_pivot.stopPivot();
        m_slide.stopSlide();
    }
}
