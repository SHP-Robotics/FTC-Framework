package org.firstinspires.ftc.teamcode.teleops.experimental;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp()
public class PixelRumble extends LinearOpMode {
    double rumbleScalar = 0.7;
    double rumbleAmount;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap);
        VisionSubsystem visionSubsystem = new VisionSubsystem(hardwareMap, "not field");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("amount not field", visionSubsystem.getAmountNotField());
            telemetry.update();

            mecanumController.drive(gamepad1);
            rumbleAmount = Math.min(1, visionSubsystem.getAmountNotField()*rumbleScalar);
            gamepad1.rumble(rumbleAmount, rumbleAmount, 300);
        }
    }
}
