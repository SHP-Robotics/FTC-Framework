package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.debug.Claw;
import org.firstinspires.ftc.teamcode.debug.LinearSlide;
import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.SpeedType;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;

@TeleOp(name = "CenterStage Driver Oriented")
public class CenterstageDriverOriented extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SpeedController speed = new SpeedController.SpeedBuilder(SpeedType.GEAR_SHIFT)
                .setGearSpacing(0.1).build();

        MecanumController mecanumController = new MecanumController(hardwareMap, speed);
        mecanumController.setDriveSpeed(1);

        LinearSlide lift = new LinearSlide(hardwareMap, true);
        lift.applyLiftBrakes();

        Claw claw = new Claw(hardwareMap, "claw");
        claw.setRange(Constants.CLAW_OPEN, Constants.CLAW_CLOSE);

        waitForStart();

        while (opModeIsActive()) {
            mecanumController.drive(gamepad1);
            lift.drive(DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.LIFT_POWER_UP)
                    - DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.LIFT_POWER_DOWN));

            if (DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.CLAW_OPEN)) {
                claw.open();
            } else if (DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.CLAW_CLOSE)) {
                claw.close();
            }
        }
    }
}
