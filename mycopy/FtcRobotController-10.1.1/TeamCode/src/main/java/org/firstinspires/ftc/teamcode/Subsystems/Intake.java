package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Constants.Constants.IntakeConstants.*;
public class Intake {

    private Servo intake;

    public Intake(HardwareMap hardwareMap){
        intake = hardwareMap.get(Servo.class, "intakeServo");
    }

    public void closeIntake(){
        intake.setPosition(INTAKE_OPEN);
    }

    public void openIntake(){
        intake.setPosition(INTAKE_CLOSE);
    }
}
