package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.OneMotorSystem;
import org.firstinspires.ftc.teamcode.debug.Side;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.SpeedType;
import org.firstinspires.ftc.teamcode.debug.Synchronous;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;

@TeleOp(name = "CenterStage Driver Oriented")
public class CenterstageDriverOriented extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedType.SINGLE_OVERRIDE)
                .setNaturalSpeed(0.5)
                .setOverrideSpeed(0.2)
                .build();

        MecanumController mecanumController = new MecanumController(hardwareMap, speedController);

        Synchronous climber = new Synchronous(hardwareMap, "leftClimber", "rightClimber");
        climber.setMotorDirection(Side.LEFT, DcMotorSimple.Direction.REVERSE);

        OneMotorSystem lift = new OneMotorSystem.OneMotorSystemBuilder(hardwareMap, "lift")
                .setDirection(DcMotorSimple.Direction.REVERSE)
                .setUseBrakes(true)
                .setUseEncoders(false)
                .setStaticPower(0.3)
                .build();

        OneMotorSystem intake = new OneMotorSystem.OneMotorSystemBuilder(hardwareMap, "intake")
                .setDirection(DcMotorSimple.Direction.REVERSE)
                .setUseEncoders(false)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            mecanumController.drive(gamepad1);
            lift.drive(DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.LIFT_POWER));

            if (gamepad1.x) {

                climber.setPowerSynchronous(DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.CLIMBER_POWER_UP)
                        - DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.CLIMBER_POWER_DOWN));

            } else {

                intake.drive(DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.INTAKE_POWER_FORWARDS)
                    - DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.INTAKE_POWER_BACKWARDS));

            }
        }
    }
}
