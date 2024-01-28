package org.firstinspires.ftc.teamcode.autos.debugging;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.debug.MecanumController;

@Disabled
@Autonomous()
public class TestEncoders extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap);
        mecanumController.setMotorsRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanumController.setMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("front Left Encoder", mecanumController.leftFront.getCurrentPosition());
            telemetry.addData("front Right Encoder", mecanumController.rightFront.getCurrentPosition());
            telemetry.addData("rear Left Encoder", mecanumController.leftRear.getCurrentPosition());
            telemetry.addData("rear Right Encoder", mecanumController.rightRear.getCurrentPosition());
            telemetry.update();
        }
    }
}
