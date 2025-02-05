package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.commands.ServoCommands;

@TeleOp(name = "Servo Test")
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo servo = (Servo) hardwareMap.get("servo");

        boolean enabled = true;
        ElapsedTime elapsedTime = new ElapsedTime();

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && elapsedTime.seconds() < 1.0) {
            elapsedTime.reset();
            enabled = !enabled;

            telemetry.addData("enabled", enabled);
            telemetry.update();

            if (enabled) {
                ServoCommands.enableServo(servo);
                servo.setPosition(1.0);
            } else
                ServoCommands.disableServo(servo);
        }
    }
}
