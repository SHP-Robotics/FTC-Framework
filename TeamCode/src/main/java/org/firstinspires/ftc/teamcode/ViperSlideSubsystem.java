package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ViperSlideSubsystem.HangMode.FINISH;
import static org.firstinspires.ftc.teamcode.ViperSlideSubsystem.HangMode.NONE;
import static org.firstinspires.ftc.teamcode.ViperSlideSubsystem.HangMode.SETUP;
import static org.firstinspires.ftc.teamcode.ViperSlideSubsystem.HangMode.VIPERDOWN;
import static org.firstinspires.ftc.teamcode.ViperSlideSubsystem.HangMode.VIPERUP;
import static org.firstinspires.ftc.teamcode.ViperSlideSubsystem.HangMode.WORMGEARBACK;
import static org.firstinspires.ftc.teamcode.ViperSlideSubsystem.HangMode.WORMGEARFOWARD;
import static org.firstinspires.ftc.teamcode.ViperSlideSubsystem.ViperMode.DRIVING;
import static org.firstinspires.ftc.teamcode.ViperSlideSubsystem.ViperMode.DRIVING2;
import static org.firstinspires.ftc.teamcode.ViperSlideSubsystem.ViperMode.INTAKE;
import static org.firstinspires.ftc.teamcode.ViperSlideSubsystem.ViperMode.OUTTAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ViperSlideSubsystem {

    public enum ViperMode {
        DRIVING (0),
        INTAKE (0),
        DRIVING2 (0),

        OUTTAKE (2650);


        ViperMode(int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }

        private final int position;
    }
    public enum HangMode {
        NONE (0),

        SETUP (900),
        VIPERDOWN (0),
        WORMGEARBACK (0),
        WORMGEARFOWARD (0),
        VIPERUP (900),
        FINISH (0);


        HangMode(int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }

        private final int position;
    }
    private DcMotor viperSlide;
    private ViperMode mode = DRIVING2;
    public static HangMode hangMode = NONE;
    ElapsedTime elapsedTime=new ElapsedTime();
    //    private  final int offset=-1540;
    public ViperSlideSubsystem(HardwareMap hardwareMap) {
        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");
        viperSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlide.setTargetPosition(0);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    public void  switchPower(){
        if (mode.getPosition()==0 && hangMode !=VIPERDOWN &&  hangMode !=WORMGEARBACK ) {
            if (viperSlide.getCurrentPosition() < 50) {
                viperSlide.setPower(0);

            }else{
                viperSlide.setPower(-0.5);

            }

        }else{
            viperSlide.setPower(1);

        }
    }
    public void update() {
        viperSlide.setTargetPosition(mode.getPosition());
        switchPower();


    }

    public void updateHanging() {
        viperSlide.setTargetPosition(hangMode.getPosition());
        viperSlide.setPower(1);

        if (hangMode==VIPERDOWN) {
            viperSlide.setPower(0.5);
        }else if (hangMode==VIPERUP) {
            viperSlide.setPower(0.1);

        }

    }

    public void updateTelemetry(Telemetry telemetry) {

        telemetry.addData("Viper Mode", mode);
        telemetry.addData("Viper Position", viperSlide.getCurrentPosition());

    }
}
