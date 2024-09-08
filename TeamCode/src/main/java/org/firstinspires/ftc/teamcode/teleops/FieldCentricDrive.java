package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.Tracker;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

@TeleOp
public class FieldCentricDrive extends OpMode {
    MecanumController mecanumController;
    Tracker tracker;
    TeleOpController teleOpController;
    Arm arm;

    @Override
    public void init() {
        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);

        arm = new Arm(hardwareMap);
    }

    @Override
    public void loop() {
        tracker.updateOdometry();
        teleOpController.updateSpeed(gamepad1);
        teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        arm.update(gamepad1);
    }
}