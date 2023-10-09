package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.debug.*;

@TeleOp(name = "Test Drive")
public class TestDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Synchronous climber = new Synchronous(hardwareMap, "leftClimber", "rightClimber");
        climber.setMotorDirection(Side.LEFT, DcMotor.Direction.FORWARD);
        climber.setMotorDirection(Side.RIGHT, DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            climber.setPowerSynchronous(gamepad1.right_stick_y);
        }
    }
}
