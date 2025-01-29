package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.utils.*;
public class Pivot{
    private DcMotorEx PO, PT;

    public Pivot(HardwareMap hardwareMap, double power){
        PO = hardwareMap.get(DcMotorEx.class, "pivotOne");
        PT = hardwareMap.get(DcMotorEx.class, "pivotTwo");

        PO.setDirection(DcMotorEx.Direction.FORWARD);
        PT.setDirection(DcMotorEx.Direction.REVERSE);
        setPower(power);
        resetEncoders();
    }

    public void stopPivot(){
        setPower(0);
    }
    public void setPower(double power){
        PO.setPower(power);
        PT.setPower(power);
    }

    private void resetEncoders(){
        reset_runWithEncoder(PO, PT);
    }
    public void pivotRun(int pos){
        if(pos != 0) {
            PO.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            PT.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }else{
            setPower(0);
        }

    }

}
