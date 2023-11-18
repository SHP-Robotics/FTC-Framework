package org.firstinspires.ftc.teamcode.centerstage.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.debug.WeightedDcMotor;
import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.WeightedMecanumController;

@TeleOp(name = "No Control")
public class NoControl extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        WeightedMecanumController weightedMecanumController = new WeightedMecanumController(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            weightedMecanumController.driveWeights(gamepad1);

            weightedMecanumController.leftFront.setPower(1);
            weightedMecanumController.rightFront.setPower(1);
            weightedMecanumController.leftRear.setPower(1);
            weightedMecanumController.rightRear.setPower(1);
        }
    }
}
