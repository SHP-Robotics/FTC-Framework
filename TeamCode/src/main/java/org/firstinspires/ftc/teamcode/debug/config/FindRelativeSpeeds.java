package org.firstinspires.ftc.teamcode.debug.config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.debug.AccumulationControlledDcMotor;
import org.firstinspires.ftc.teamcode.debug.MecanumController;

@Disabled
@Autonomous()
public class FindRelativeSpeeds extends LinearOpMode {
    MecanumController mecanumController;

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumController = new MecanumController(hardwareMap);

        mecanumController.leftFront = new AccumulationControlledDcMotor.AccumulationControlledDcMotorBuilder(mecanumController.leftFront)
                .setkP(0.7/0.976)
                .setGamma(0)
                .build();

        mecanumController.rightFront = new AccumulationControlledDcMotor.AccumulationControlledDcMotorBuilder(mecanumController.rightFront)
                .setkP(0.7/0.992)
                .setGamma(0)
                .build();

        mecanumController.leftRear = new AccumulationControlledDcMotor.AccumulationControlledDcMotorBuilder(mecanumController.leftRear)
                .setkP(0.7/0.992)
                .setGamma(0)
                .build();

        mecanumController.rightRear = new AccumulationControlledDcMotor.AccumulationControlledDcMotorBuilder(mecanumController.rightRear)
                .setkP(0.7)
                .setGamma(0)
                .build();

        mecanumController.setMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanumController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double maxLF = 0;
        double maxRF = 0;
        double maxLR = 0;
        double maxRR = 0;

        mecanumController.leftFront.setPower(1);
        mecanumController.rightFront.setPower(1);
        mecanumController.leftRear.setPower(1);
        mecanumController.rightRear.setPower(1);

        while (!opModeIsActive()) {
            if (mecanumController.leftFront instanceof AccumulationControlledDcMotor) {
                double veloLF = ((AccumulationControlledDcMotor) mecanumController.leftFront).getVelocity(AngleUnit.RADIANS);
                if (veloLF > maxLF) {
                    maxLF = veloLF;
                }
            }

            if (mecanumController.rightFront instanceof AccumulationControlledDcMotor) {
                double veloRF = ((AccumulationControlledDcMotor) mecanumController.rightFront).getVelocity(AngleUnit.RADIANS);
                if (veloRF > maxRF) {
                    maxRF = veloRF;
                }
            }

            if (mecanumController.leftRear instanceof AccumulationControlledDcMotor) {
                double veloLR = ((AccumulationControlledDcMotor) mecanumController.leftRear).getVelocity(AngleUnit.RADIANS);
                if (veloLR > maxLR) {
                    maxLR = veloLR;
                }
            }

            if (mecanumController.rightRear instanceof AccumulationControlledDcMotor) {
                double veloRR = ((AccumulationControlledDcMotor) mecanumController.rightRear).getVelocity(AngleUnit.RADIANS);
                if (veloRR > maxRR) {
                    maxRR = veloRR;
                }
            }
        }

        waitForStart();

        mecanumController.leftFront.setPower(0);
        mecanumController.rightFront.setPower(0);
        mecanumController.leftRear.setPower(0);
        mecanumController.rightRear.setPower(0);

        double maxVelo = Math.max(maxLF, Math.max(maxRF, Math.max(maxLR, maxRR)));

        telemetry.addData("maxVeloLF", maxLF);
        telemetry.addData("maxVeloRF", maxRF);
        telemetry.addData("maxVeloLR", maxLR);
        telemetry.addData("maxVeloRR", maxRR);
        telemetry.addData("relativeVeloLF", maxLF/maxVelo);
        telemetry.addData("relativeVeloRF", maxRF/maxVelo);
        telemetry.addData("relativeVeloLR", maxLR/maxVelo);
        telemetry.addData("relativeVeloRR", maxRR/maxVelo);
        telemetry.update();

        sleep(10000);
    }
}
