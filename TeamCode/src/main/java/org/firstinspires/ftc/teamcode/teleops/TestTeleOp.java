package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.R;

@Disabled
@TeleOp(name = "Test Tele Op")
public class TestTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo air = hardwareMap.get(Servo.class, "air");
        air.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            air.setPosition(gamepad1.left_stick_y);
            telemetry.addData("position", gamepad1.left_stick_y);
            telemetry.update();
            sleep(20);
        }
    }
}
