package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Ultrasonic")
public class Ultrasonic extends LinearOpMode {
    AnalogInput distanceSensor1;
    AnalogInput distanceSensor2;

    public double ff(double x) {
        return x * 3200;
    }

    @Override
    public void runOpMode() {
        distanceSensor1 = hardwareMap.get(AnalogInput.class, "ultra1");
        distanceSensor2 = hardwareMap.get(AnalogInput.class, "ultra2");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("U1, Voltage", distanceSensor1.getVoltage());
            telemetry.addData("U1, Meters", ff(distanceSensor1.getVoltage()));

            telemetry.addLine();
            
            telemetry.addData("U2, Voltage", distanceSensor2.getVoltage());
            telemetry.addData("U2, Meters", ff(distanceSensor2.getVoltage()));

            telemetry.update();
        }
    }
}
