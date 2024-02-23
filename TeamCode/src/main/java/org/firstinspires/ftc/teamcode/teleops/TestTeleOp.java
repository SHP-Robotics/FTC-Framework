package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(name = "Test Tele Op")
public class TestTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo outtake = hardwareMap.get(Servo.class, "outtake");
        outtake.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            outtake.setPosition(gamepad1.left_stick_y);
            telemetry.addData("position", gamepad1.left_stick_y);
            telemetry.update();
            sleep(20);
        }
    }
}
