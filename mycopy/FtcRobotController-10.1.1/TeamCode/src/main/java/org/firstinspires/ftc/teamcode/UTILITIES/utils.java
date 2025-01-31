package org.firstinspires.ftc.teamcode.UTILITIES;

import com.qualcomm.robotcore.hardware.DcMotor;

public class utils{
    public static void reset_runWithoutEncoder(DcMotor one) {
        one.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        one.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public static void reset_runWithEncoder(DcMotor one){
        one.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public static void setRunWithoutEncoder(DcMotor one){
        one.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public static void setRunWithEncoder(DcMotor one){
        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public static void setRunWithoutEncoder(DcMotor one, DcMotor two){
        one.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public static void setRunWithEncoder(DcMotor one, DcMotor two){
        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public static void reset_runWithoutEncoder(DcMotor one, DcMotor two) {
        one.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        two.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        one.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public static void reset_runWithEncoder(DcMotor one, DcMotor two){
        one.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        two.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    }

