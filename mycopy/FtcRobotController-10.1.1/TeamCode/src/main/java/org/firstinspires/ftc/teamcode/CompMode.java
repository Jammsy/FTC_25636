package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static org.firstinspires.ftc.teamcode.Constants.Constants.SlideConstants.*;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Subsystems.Slide;
import static org.firstinspires.ftc.teamcode.Constants.Constants.PivotConstants.*;
@TeleOp(name = "CompMode", group = "Iterative Opmode")
public class CompMode extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain m_drive = null;
    private Intake m_intake = null;
    private Slide m_slide = null;
    private Pivot m_pivot = null;
    private ExecutorService executor = Executors.newFixedThreadPool(4);
    @Override
    public void init() {
        m_drive = new Drivetrain(hardwareMap);
        m_intake = new Intake(hardwareMap);
        m_slide = new Slide(hardwareMap);
        m_pivot = new Pivot(hardwareMap, pivotSpeed);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start(){
        runtime.reset();
    }

    @Override
    public void loop() {
        /*m_drive.drive_Cartesian(gamepad1.left_stick_x*1.1, -gamepad1.left_stick_y*1.1, gamepad1.right_stick_x*1.1);
        if (gamepad1.dpad_right) m_pivot.pivotRun(HIGH_RUNG);
        if (gamepad1.dpad_down) m_pivot.pivotRun(GROUND);
        if (gamepad1.dpad_left) m_pivot.pivotRun(CLIMB);
        if (gamepad1.dpad_up) m_pivot.pivotRun(SUB);
        if (gamepad1.circle) m_pivot.pivotRun(ZERO);

        if(gamepad1.triangle) m_intake.closeIntake(); else m_intake.openIntake();

        if(gamepad1.right_bumper) m_slide.slideOut();
        if(gamepad1.left_bumper) m_slide.slideIn();*/
        executor.submit(() -> m_drive.drive_Cartesian(gamepad1.left_stick_x * 1.1, -gamepad1.left_stick_y * 1.1, gamepad1.right_stick_x * 1.1));

        // Pivot Control (logic moved OUTSIDE the Runnable)
        if (gamepad1.dpad_right) executor.submit(() -> m_pivot.pivotRun(HIGH_RUNG));
        if (gamepad1.dpad_down) executor.submit(() -> m_pivot.pivotRun(GROUND));
        if (gamepad1.dpad_left) executor.submit(() -> m_pivot.pivotRun(CLIMB));
        if (gamepad1.dpad_up) executor.submit(() -> m_pivot.pivotRun(SUB));
        if (gamepad1.circle) executor.submit(() -> m_pivot.pivotRun(ZERO));

        // Intake Control (logic moved OUTSIDE the Runnable)
        if (gamepad1.triangle) executor.submit(() -> m_intake.closeIntake());
        else executor.submit(() -> m_intake.openIntake());

        // Slide Control (logic moved OUTSIDE the Runnable)
        if (gamepad1.right_bumper) executor.submit(() -> m_slide.slideOut(SLIDE_POWER));
        if (gamepad1.left_bumper) executor.submit(() -> m_slide.slideIn(SLIDE_POWER));
    }

    @Override
    public void stop(){
        executor.shutdown();
        m_drive.stopDriving();
        m_pivot.stopPivot();
        m_slide.stopSlide();
    }
}
