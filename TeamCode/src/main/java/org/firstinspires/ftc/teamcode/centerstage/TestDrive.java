package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
<<<<<<< Updated upstream
import com.qualcomm.robotcore.hardware.DcMotor;
=======
import com.qualcomm.robotcore.hardware.DcMotorSimple;
>>>>>>> Stashed changes

import org.firstinspires.ftc.teamcode.debug.*;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;

@TeleOp(name = "Test Drive")
public class TestDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
<<<<<<< Updated upstream
        Synchronous climber = new Synchronous(hardwareMap, "leftClimber", "rightClimber");
        climber.setMotorDirection(Side.LEFT, DcMotor.Direction.FORWARD);
        climber.setMotorDirection(Side.RIGHT, DcMotor.Direction.REVERSE);
=======
        //MecanumController mecanumController = new MecanumController(hardwareMap, Speed.PID_CONTROLLED_WITH_OVERRIDE);
        //mecanumController.setDriveSpeed(0.7);

        Synchronous climber = new Synchronous(hardwareMap, "leftClimber", "rightClimber");
        climber.setMotorDirection(Side.LEFT, DcMotorSimple.Direction.REVERSE);
        climber.setMotorDirection(Side.RIGHT, DcMotorSimple.Direction.FORWARD);
>>>>>>> Stashed changes

        waitForStart();

        while (opModeIsActive()) {
<<<<<<< Updated upstream
=======
            //mecanumController.drive(gamepad1);
            //telemetry.addData("up gear", DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.GEAR_UP));
            //telemetry.update();
>>>>>>> Stashed changes
            climber.setPowerSynchronous(gamepad1.right_stick_y);
        }
    }
}
