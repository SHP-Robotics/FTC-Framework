package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.Side;
import org.firstinspires.ftc.teamcode.debug.Synchronous;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;

@Disabled
@TeleOp(name = "CenterStage Driver Oriented")
public class CenterstageDriverOriented extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumController mecanumController = new MecanumController(hardwareMap);
        mecanumController.setDriveSpeed(0.7);

        Synchronous climber = new Synchronous(hardwareMap, "leftClimber", "rightClimber");
        climber.setMotorDirection(Side.LEFT, DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            mecanumController.drive(gamepad1);
            climber.setPowerSynchronous(DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.CLIMBER_POWER));
        }
    }
}
