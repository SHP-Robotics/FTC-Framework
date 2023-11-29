package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.debug.MecanumController;

@Autonomous(name = "Test Auto")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        /*
        CRServo planeLauncher = hardwareMap.get(CRServo.class, "planeLauncher");

        waitForStart();

        planeLauncher.setPower(1);

        sleep(1000);

         */


        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensorForward");
        colorSensor.enableLed(true);

        boolean detectedBlue = false;

        waitForStart();

        while (opModeIsActive()) {
            if (detectedBlue) {
                if (colorSensor.blue() > 1100) {
                    telemetry.addLine("left");
                    telemetry.update();
                } else {
                    break;
                }
            } else {
                if (colorSensor.blue() > 1100) {
                    detectedBlue = true;
                } else {
                    telemetry.addLine("right");
                    telemetry.update();
                }
            }
        }


        /*

        MecanumController mecanumController = new MecanumController(hardwareMap);

        waitForStart();

        mecanumController.leftFront.setPower(1);
        sleep(1000);
        mecanumController.rightFront.setPower(1);
        sleep(1000);
        mecanumController.leftRear.setPower(1);
        sleep(1000);
        mecanumController.rightRear.setPower(1);
        sleep(1000);
        */
    }
}
