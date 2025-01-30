package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import static org.firstinspires.ftc.teamcode.utils.*;

public class Slide {

    private DcMotorEx SL, SR;
    private TouchSensor slideLimit;

    public Slide(HardwareMap hardwareMap){
        SL = hardwareMap.get(DcMotorEx.class, "linSlideLeft");
        SR = hardwareMap.get(DcMotorEx.class, "linSlideRight");
        slideLimit = hardwareMap.get(TouchSensor.class, "slideTouchLimit");


        SR.setDirection(DcMotorEx.Direction.FORWARD);
        SL.setDirection(DcMotorEx.Direction.REVERSE);

        reset_runWithoutEncoder(SL, SR);
        resetEncoders();
    }

    public void slideOut(double power){
        if(power !=0) {
                setPower(-(power));
        }
        stopSlide();
    }

    public void slideIn(double power){
        if(power != 0) {
            if (!slideLimit.isPressed()) {
                setPower(power);
            } else {
                stopSlide();
            }
        }
        stopSlide();
    }

    public void slideClimb(){
        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(!slideLimit.isPressed()){
            setPower(1);
        }
    }
    public void resetEncoders(){
        reset_runWithoutEncoder(SL, SR);
    }

    public void stopSlide(){
        setPower(0);
    }

    public void setPower(double power){
        SL.setPower(power);
        SR.setPower(power);
    }
}
