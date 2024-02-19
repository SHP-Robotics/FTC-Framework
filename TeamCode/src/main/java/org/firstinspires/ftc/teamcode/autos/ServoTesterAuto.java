package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shplib.Constants;

@Disabled
@Autonomous
public class ServoTesterAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = (Servo) hardwareMap.get("35kg");
        ElapsedTime elapsedTime = new ElapsedTime();
        boolean down = true;
        servo.setPosition(Constants.Intake.kWristDown);

        waitForStart();
        elapsedTime.reset();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Servo position", servo.getPosition());
            telemetry.update();

            if (elapsedTime.seconds() > 3) {
                servo.setPosition(down ? Constants.Intake.kWristHalfway : Constants.Intake.kWristDown);
                down = !down;
                elapsedTime.reset();
            }
        }
    }
}
