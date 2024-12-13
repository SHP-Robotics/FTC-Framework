package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.ViperSlideSubsystem.*;
import static org.firstinspires.ftc.teamcode.ViperSlideSubsystem.HangMode.FINISH;
import static org.firstinspires.ftc.teamcode.ViperSlideSubsystem.HangMode.VIPERUP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.devices.GamepadKey;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;



@TeleOp
public class DontPressSquare extends LinearOpMode {
    MecanumController mecanumController;
    DeterministicTracker tracker;
    TeleOpController teleOpController;
    ViperSlideSubsystem viperSlideSubsystem;
    ClawSubsystem clawSubsystem;
    WristSubsystem wristSubsystem;

    WormGearSubsystem wormGearSubsystem;
    GamepadInterface gamepadInterface;
    TouchSensor touchSensor;

    @Override
    public void runOpMode() {
        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);

        viperSlideSubsystem = new ViperSlideSubsystem(hardwareMap);
        clawSubsystem = new ClawSubsystem(hardwareMap);
        wristSubsystem = new WristSubsystem(hardwareMap);
        wormGearSubsystem = new WormGearSubsystem(hardwareMap);
        wormGearSubsystem.reset();

        gamepadInterface = new GamepadInterface(gamepad1);

        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        waitForStart();
        teleOpController.resetIMU();
        tracker.reset();
        while (opModeIsActive()) {
            telemetry.addData("X", tracker.getCurrentPosition().getX());
            telemetry.addData("Y", tracker.getCurrentPosition().getY());
            telemetry.addData("HeadingRadians", tracker.getCurrentPosition().getHeadingRadians());

            tracker.update();
            gamepadInterface.update();

            teleOpController.updateSpeed(gamepad1);
            if (gamepad1.right_trigger > 0.9) {

                teleOpController.driveFieldCentric(gamepad1.left_stick_y * 0.2, -gamepad1.left_stick_x * 0.2, -gamepad1.right_stick_x * 0.2);
            } else {
                teleOpController.driveFieldCentric(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            }
            if (gamepad1.x) {
                wormGearSubsystem.setToZero(touchSensor, telemetry );
            }

            if (gamepad1.b) {
                teleOpController.resetIMU();
                tracker.reset();
            }

            if (gamepadInterface.isKeyUp(GamepadKey.LEFT_BUMPER)) {

                wormGearSubsystem.cycle();
                viperSlideSubsystem.cycle();


                wormGearSubsystem.update();
                viperSlideSubsystem.update();

            }
            if (gamepadInterface.isKeyUp(GamepadKey.RIGHT_BUMPER)) {

                wormGearSubsystem.cycleHanging();
                viperSlideSubsystem.cycleHanging();
                wormGearSubsystem.updateHanging();
                viperSlideSubsystem.updateHanging();
                if (hangMode == VIPERUP || hangMode == FINISH) {
                    teleOpController.driveFieldCentric(1, 0, 0);

                }


                clawSubsystem.update();
                wristSubsystem.update();
                wormGearSubsystem.updateTelemetry(telemetry);
                viperSlideSubsystem.updateTelemetry(telemetry);
                clawSubsystem.updateTelemetry(telemetry);
                wristSubsystem.updateTelemetry(telemetry);
                telemetry.update();
            }

        }
    }
}

