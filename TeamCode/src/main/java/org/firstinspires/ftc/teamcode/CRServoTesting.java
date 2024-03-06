package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

//@Disabled
@TeleOp
public class CRServoTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo servo = (CRServo) hardwareMap.get("servo");
        servo.setDirection(CRServo.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            servo.setPower(gamepad1.left_stick_y);

            telemetry.addLine("Name: Servo");
            telemetry.addLine("Direction: FORWARD");
            telemetry.update();
        }
    }
}
