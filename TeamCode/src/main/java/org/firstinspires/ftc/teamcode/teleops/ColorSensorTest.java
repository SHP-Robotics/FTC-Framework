package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Disabled
@TeleOp()
public class ColorSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor testSensor = (ColorSensor) hardwareMap.get("colorSensor");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("r", testSensor.red());
            telemetry.addData("g", testSensor.green());
            telemetry.addData("b", testSensor.blue());
            telemetry.update();
        }
    }
}
