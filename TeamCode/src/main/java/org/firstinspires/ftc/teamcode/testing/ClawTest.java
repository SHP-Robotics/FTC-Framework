package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.ClawSubsystem.ClawState.CLOSE;
import static org.firstinspires.ftc.teamcode.ClawSubsystem.ClawState.OPEN;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ClawSubsystem;

@TeleOp(name = "Claw Test")
public class ClawTest extends LinearOpMode {
    private ClawSubsystem clawSubsystem;
//    private Servo servo;
    private ElapsedTime elapsedTime;

    @Override
    public void runOpMode() {
        clawSubsystem = new ClawSubsystem(hardwareMap);
//        servo = (Servo) hardwareMap.get("intake");

        elapsedTime = new ElapsedTime();

        waitForStart();

        elapsedTime.reset();

        while (opModeIsActive()) {
            loopOpMode();
        }
    }

    public void loopOpMode() {
        if (gamepad1.left_trigger > 0.9)
            clawSubsystem.setState(OPEN);
        else if (gamepad1.right_trigger > 0.1)
            clawSubsystem.setState(CLOSE);

        clawSubsystem.update();
        clawSubsystem. updateTelemetry(telemetry);

//        if (gamepad1.right_bumper)
//            servo.setPosition(-gamepad1.right_stick_y);

//        telemetry.addData("position", servo.getPosition());

        telemetry.addData("Loop Times", elapsedTime.milliseconds());
        elapsedTime.reset();
        telemetry.update();
    }
}
