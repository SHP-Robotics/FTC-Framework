package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.shprobotics.pestocore.devices.GamepadInterface;

import org.firstinspires.ftc.teamcode.SampleSlideSubsystem;

@TeleOp
public class TestTeleOp extends LinearOpMode {
    /**
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadInterface gamepadInterface = new GamepadInterface(gamepad1);
        SampleSlideSubsystem sampleSlideSubsystem = new SampleSlideSubsystem(hardwareMap);
        sampleSlideSubsystem.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            gamepadInterface.update();

            telemetry.addData("position", sampleSlideSubsystem.getPosition());
            telemetry.update();
        }
    }
}
