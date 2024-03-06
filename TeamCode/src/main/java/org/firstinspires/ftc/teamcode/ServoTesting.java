package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp
public class ServoTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = (Servo) hardwareMap.get("servo");
        servo.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(gamepad1.left_stick_y);

            telemetry.addLine("Name: Servo");
            telemetry.addLine("Direction: FORWARD");
            telemetry.addData("Position", servo.getPosition());
            telemetry.update();
        }
    }
}
