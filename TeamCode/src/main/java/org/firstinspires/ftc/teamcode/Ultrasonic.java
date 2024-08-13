package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Ultrasonic")
public class Ultrasonic extends LinearOpMode {
    AnalogInput distanceSensor;

    public double ff(double x) {
        return x * 3200;
    }

    @Override
    public void runOpMode() {
        distanceSensor = hardwareMap.get(AnalogInput.class, "frontUltra");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Voltage", distanceSensor.getVoltage());
            telemetry.addData("Meters", ff(distanceSensor.getVoltage()));

            telemetry.update();
        }
    }
}
