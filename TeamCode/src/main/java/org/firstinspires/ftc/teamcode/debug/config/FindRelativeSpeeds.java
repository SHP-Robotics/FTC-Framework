package org.firstinspires.ftc.teamcode.debug.config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.debug.AccumulationControlledDcMotor;

@Disabled
@Autonomous()
public class FindRelativeSpeeds extends LinearOpMode {
    MecanumController mecanumController;

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumController = new MecanumController(hardwareMap);

        for (int i = 0; i < 4; i++) {
            mecanumController.motors[i] = new AccumulationControlledDcMotor.AccumulationControlledDcMotorBuilder(mecanumController.motors[i])
                    .setkP(0.7/Constants.powers[i])
                    .setGamma(0)
                    .build();
        }

        mecanumController.setMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanumController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double[] velos = new double[]{0, 0, 0, 0};
        double[] maxVelos = new double[]{0, 0, 0, 0};

        mecanumController.motors[0].setPower(1);
        mecanumController.motors[1].setPower(1);
        mecanumController.motors[2].setPower(1);
        mecanumController.motors[3].setPower(1);

        while (!opModeIsActive()) {
            for (int i = 0; i < 4; i++) {
                if (mecanumController.motors[i] instanceof AccumulationControlledDcMotor) {
                    velos[i] = ((AccumulationControlledDcMotor) mecanumController.motors[i]).getVelocity(AngleUnit.RADIANS);
                    if (velos[i] > maxVelos[i]) {
                        maxVelos[i] = velos[i];
                    }
                }
            }
        }

        waitForStart();

        mecanumController.deactivate();

        double maxVelo = 0;
        for (double velo: maxVelos) {
            if (velo > maxVelo) {
                maxVelo = velo;
            }
        }

        telemetry.addData("maxVeloLF", maxVelos[0]);
        telemetry.addData("maxVeloRF", maxVelos[1]);
        telemetry.addData("maxVeloLR", maxVelos[2]);
        telemetry.addData("maxVeloRR", maxVelos[3]);
        telemetry.addData("relativeVeloLF", maxVelos[0]/maxVelo);
        telemetry.addData("relativeVeloRF", maxVelos[1]/maxVelo);
        telemetry.addData("relativeVeloLR", maxVelos[2]/maxVelo);
        telemetry.addData("relativeVeloRR", maxVelos[3]/maxVelo);
        telemetry.update();

        sleep(10000);
    }
}
