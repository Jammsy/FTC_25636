package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static org.firstinspires.ftc.teamcode.Constants.Constants.SlideConstants.*;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Subsystems.Slide;
import static org.firstinspires.ftc.teamcode.utils.*;
import static org.firstinspires.ftc.teamcode.Constants.Constants.PivotConstants.*;
@TeleOp(name = "CompMode", group = "Iterative Opmode")
public class CompMode extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain m_drive = null;
    private Intake m_intake = null;
    private Slide m_slide = null;
    private Pivot m_pivot = null;
    private RevBlinkinLedDriver LED = null;
    Map<String, RevBlinkinLedDriver.BlinkinPattern> ledStates= new HashMap<String,RevBlinkinLedDriver.BlinkinPattern>();

    @Override
    public void init() {
        m_drive = new Drivetrain(hardwareMap);
        m_intake = new Intake(hardwareMap);
        m_slide = new Slide(hardwareMap);
        m_pivot = new Pivot(hardwareMap);
        LED = hardwareMap.get(RevBlinkinLedDriver.class, "LED");

        ledStates.put("Init", RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
        ledStates.put("Intake", RevBlinkinLedDriver.BlinkinPattern.GREEN);
        ledStates.put("Score", RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        ledStates.put("Rainbow", RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        ledStates.put("Off", RevBlinkinLedDriver.BlinkinPattern.BLACK);

        LED.setPattern(ledStates.get("Init"));

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start(){
        runtime.reset();
    }

    @Override
    public void loop() {
        // Drive control
        m_drive.drive_Cartesian(gamepad1.left_stick_x * 1.1, -gamepad1.left_stick_y * 1.1, -gamepad1.right_stick_x * 1.1);

        // Pivot control
        if (gamepad1.right_trigger > 0.3) m_pivot.pivotRun(HIGH_RUNG);
        else if (gamepad1.dpad_down) m_pivot.pivotRun(GROUND);
        else if (gamepad1.dpad_left) m_pivot.pivotRun(BASKET);
        else if (gamepad1.dpad_up) m_pivot.pivotRun(SUB);
        else if (gamepad1.circle) m_pivot.pivotRun(ZERO);
        else if(gamepad1.left_trigger > 0.3) m_pivot.pivotRun(WALL);

        //ResetEncoders
        if (gamepad1.touchpad) m_pivot.resetEncoders();

        // Intake control
        if (gamepad1.triangle){m_intake.closeIntake(); LED.setPattern(ledStates.get("Intake"));}
        else{ m_intake.openIntake(); LED.setPattern(ledStates.get("Rainbow"));}

        // Slide control
        if (gamepad1.right_bumper) m_slide.slideOut(SLIDE_POWER);
        else if (gamepad1.left_bumper) m_slide.slideIn(SLIDE_POWER);
    }

    @Override
    public void stop(){
        m_drive.stopDriving();
        m_pivot.stopPivot();
        m_slide.stopSlide();
    }
}
