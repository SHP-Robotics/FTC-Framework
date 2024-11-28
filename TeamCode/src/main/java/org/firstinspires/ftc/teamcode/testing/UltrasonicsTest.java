package org.firstinspires.ftc.teamcode.testing;

import static java.lang.Math.max;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.MecanumController;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

@TeleOp
public class UltrasonicsTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        AnalogInput ultrasonicLeft = (AnalogInput) hardwareMap.get("ultrasonicRight");
        AnalogInput ultrasonicRight = (AnalogInput) hardwareMap.get("ultrasonicLeft");

        ColorSensor colorLeft = (ColorSensor) hardwareMap.get("colorLeft");
        ColorSensor colorRight = (ColorSensor) hardwareMap.get("colorRight");

        PID ultrasonicPID = new PID(0.02, 0.02, 0);
        ultrasonicPID.setMaxIntegralProportionRatio(0.24);
        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double avg = (ultrasonicLeft.getVoltage() + ultrasonicRight.getVoltage()) / 2;
            avg *= 62.039;
            if (avg > 48)
                continue;

            if (colorLeft.alpha() < 100 || colorRight.alpha() < 100)
                mecanumController.drive(max(-ultrasonicPID.getOutput(avg, 0), -0.3), 0, 0);
            else
                mecanumController.drive(0, 0, 0);

            telemetry.addData("l", ultrasonicLeft.getVoltage()*62.039);
            telemetry.addData("r", ultrasonicRight.getVoltage()*62.039);

            telemetry.addData("la", colorLeft.alpha());
            telemetry.addData("ra", colorRight.alpha());

            telemetry.update();
        }
    }
}
