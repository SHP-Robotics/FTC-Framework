package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.*;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.SETUP;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.VIPERDOWN;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.WORMGEARBACK;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.WormMode.DRIVING;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.WormMode.INTAKE;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.WormMode.OUTTAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class WormGearSubsystem {
    private static final int OFFSET =-1550;
    boolean  zeroed=false;

    public enum WormMode {
        DRIVING (OFFSET),
        INTAKE (-3650),
        OUTTAKE (OFFSET);



        WormMode(int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }

        private final int position;
    }
    public enum HangMode {

        NONE (0),
        SETUP (OFFSET-150),
        VIPERDOWN (OFFSET-150),
        WORMGEARBACK (-100);

        HangMode(int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }

        private final int position;
    }

    private DcMotor wormGear;
    private WormMode mode= DRIVING;
    private HangMode hangMode = NONE;

    public WormGearSubsystem(HardwareMap hardwareMap) {
        wormGear = hardwareMap.get(DcMotor.class, "WormGear");

    }

    public void cycle() {
        switch (mode) {
            case DRIVING:
                mode = INTAKE;
                break;
            case INTAKE:
                mode = OUTTAKE;
                break;
            case OUTTAKE:
                mode = DRIVING;
                break;

        }

    }
    public void cycleHanging() {
        switch (hangMode) {
            case NONE:
                hangMode = SETUP;
                break;
            case SETUP:
                hangMode = VIPERDOWN;
                break;
            case VIPERDOWN:
                hangMode = WORMGEARBACK;
                break;
            case WORMGEARBACK:
                hangMode = SETUP;
                break;

        }

    }

    public void reset() {
        wormGear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormGear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setToZero(TouchSensor touchSensor) {
        wormGear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wormGear.setPower(2);
        while(!touchSensor.isPressed()){
        }
        reset();

        wormGear.setPower(0);
        wormGear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wormGear.setTargetPosition(OFFSET);
        wormGear.setPower(2);
        while(wormGear.getCurrentPosition()>-1560){
        }
        zeroed=true;


    }
    public void update() {
        if(zeroed  && hangMode== NONE){


        wormGear.setTargetPosition(mode.getPosition());
        wormGear.setPower(0.5);
        }
    }


    public void updateHanging() {
        wormGear.setTargetPosition(hangMode.getPosition());

    }


    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("mode", mode);
        telemetry.addData("WormGear Position", wormGear.getCurrentPosition());
    }
}
