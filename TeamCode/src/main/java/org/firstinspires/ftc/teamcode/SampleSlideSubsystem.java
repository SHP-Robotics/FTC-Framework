package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static java.lang.Double.min;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SampleSlideSubsystem {
    public DcMotorEx motor;
    public static double maxPosition = 1500;

    public SampleSlideSubsystem(HardwareMap hardwareMap) {
        motor = (DcMotorEx) hardwareMap.get("sampleSlide");
        motor.setMode(STOP_AND_RESET_ENCODER);
        motor.setMode(RUN_USING_ENCODER);
    }

    public void setPower(double power) {
        if (this.motor.getCurrentPosition() > maxPosition)
            this.motor.setPower(min(power, 0));
        else
            this.motor.setPower(power);
    }

    public double getPower() {
        return this.motor.getPower();
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("SlidePower", this.getPower());
        telemetry.addData("SlidePosition", this.motor.getCurrentPosition());
    }
}
