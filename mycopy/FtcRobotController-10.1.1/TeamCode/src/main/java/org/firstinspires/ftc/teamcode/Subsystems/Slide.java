package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import static org.firstinspires.ftc.teamcode.utils.*;
import static org.firstinspires.ftc.teamcode.Constants.SlideConstants.*;

public class Slide {

    private DcMotorEx SL, SR;
    private TouchSensor slideLimit;

    public Slide(HardwareMap hardwareMap){
        SL = hardwareMap.get(DcMotorEx.class, "linSlideLeft");
        SR = hardwareMap.get(DcMotorEx.class, "linSlideRight");
        slideLimit = hardwareMap.get(TouchSensor.class, "slideTouchLimit");

        SR.setDirection(DcMotorEx.Direction.FORWARD);
        SL.setDirection(DcMotorEx.Direction.REVERSE);
        resetEncoders();
    }

    public void slideOut(){
        if(SL.getCurrentPosition() > -5000 && SR.getCurrentPosition() > -5000){
            setPower(-0.8);
        }else{
            stopSlide();
        }
    }

    public void slideIn(){
        if(!slideLimit.isPressed()){
            setPower(0.8);
        }else{
            stopSlide();
        }
    }

    private void resetEncoders(){
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
