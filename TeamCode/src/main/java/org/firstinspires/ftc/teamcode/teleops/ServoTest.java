package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@TeleOp
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = new CachingServo((Servo) hardwareMap.get("claw"));
        claw.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            claw.setPosition(-gamepad1.left_stick_y);
            telemetry.addData("val", -gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
