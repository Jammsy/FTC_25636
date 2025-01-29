package org.firstinspires.ftc.teamcode.Subsystems;

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
        resetEncoders();
    }

    public void slideOut(double power){
        if(power !=0) {
            if (SL.getCurrentPosition() > -5000 && SR.getCurrentPosition() > -5000) {
                setPower(-(power));
            } else {
                stopSlide();
            }
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
