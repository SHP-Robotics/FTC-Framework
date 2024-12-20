package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.FINISH;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.NONE;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.SETUP;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.VIPERDOWN;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.VIPERUP;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.WORMGEARBACK;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.WORMGEARFOWARD;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.WormMode.DRIVING;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.WormMode.DRIVING2;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.WormMode.INTAKE;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.WormMode.OUTTAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class WormGearSubsystem {
    private static final int OFFSET =-1550;
    boolean  zeroed=false;

    public enum WormMode {
        DRIVING (OFFSET),
        INTAKE (-3450),
        DRIVING2 (OFFSET),

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
        WORMGEARBACK (-100),
        WORMGEARFOWARD (0),
        VIPERUP (0),
        FINISH (OFFSET);
        HangMode(int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }

        private final int position;
    }

    private DcMotorEx wormGear;
    private WormMode mode= DRIVING;
    private HangMode hangMode = NONE;

    public WormGearSubsystem(HardwareMap hardwareMap) {
        wormGear = hardwareMap.get(DcMotorEx.class, "WormGear");

    }

    public void cycle() {
        switch (mode) {
            case DRIVING:
                mode = INTAKE;
                break;
            case INTAKE:
                mode = DRIVING2;
                break;
            case DRIVING2:
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
                hangMode = WORMGEARFOWARD;
                break;
            case WORMGEARFOWARD:
                hangMode = VIPERUP;
                break;
            case VIPERUP:
                hangMode = FINISH;
                break;
            case FINISH:
                hangMode = NONE;
                break;
        }

    }

    public void reset() {
        wormGear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormGear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setToZero(TouchSensor touchSensor, Telemetry telemetry) {
        wormGear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wormGear.setPower(1);
        while(!touchSensor.isPressed()){
//        while (!touchSensor.isPressed()) {
            updateTelemetry(telemetry);
            telemetry.update();
        }
        reset();
        updateTelemetry(telemetry);
        telemetry.update();
        wormGear.setPower(0);

        wormGear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wormGear.setTargetPosition(OFFSET);
        wormGear.setPower(1);
        while(wormGear.getCurrentPosition()>-1560){
        }
        zeroed=true;


    }
    public void update() {
//        if(zeroed  && hangMode== NONE){


        wormGear.setTargetPosition(mode.getPosition());
        wormGear.setPower(1);
//        }
    }


    public void updateHanging() {
        wormGear.setTargetPosition(hangMode.getPosition());
        wormGear.setPower(0.5);

    }


    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("mode", mode);
        telemetry.addData("WormGear Position", wormGear.getCurrentPosition());
        telemetry.addData("current", wormGear.getCurrent(CurrentUnit.AMPS));
    }

}
