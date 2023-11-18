package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.debug.Claw;
import org.firstinspires.ftc.teamcode.debug.LinearSlide;
import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.Side;
import org.firstinspires.ftc.teamcode.debug.Speed;
import org.firstinspires.ftc.teamcode.debug.Synchronous;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;

@TeleOp(name = "CenterStage Driver Oriented")
public class CenterstageDriverOriented extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap, Speed.SINGLE_OVERRIDE);
        mecanumController.setDriveSpeed(1);

        LinearSlide lift = new LinearSlide(hardwareMap, true);
        lift.applyLiftBrakes();

        Claw claw = new Claw(hardwareMap, "claw");
        claw.setRange(0, 1);

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
