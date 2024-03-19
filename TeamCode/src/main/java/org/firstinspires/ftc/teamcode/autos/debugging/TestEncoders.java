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
            telemetry.addData("front Left Encoder", mecanumController.motors[0].getCurrentPosition());
            telemetry.addData("front Right Encoder", mecanumController.motors[1].getCurrentPosition());
            telemetry.addData("rear Left Encoder", mecanumController.motors[2].getCurrentPosition());
            telemetry.addData("rear Right Encoder", mecanumController.motors[3].getCurrentPosition());
            telemetry.update();
        }
    }
}
