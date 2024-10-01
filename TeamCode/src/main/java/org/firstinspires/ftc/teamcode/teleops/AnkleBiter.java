package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

@TeleOp
public class AnkleBiter extends BaseRobot {
    private double debounce;
    SHPMotor motorLeft, motorRight;

    @Override
    public void init() {
        super.init();

        // Default command runs when no other commands are scheduled for the subsystem

        motorLeft = new SHPMotor(hardwareMap,"leftWheel");
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight = new SHPMotor(hardwareMap,"rightWheel");
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void start() {
        super.start();

        debounce = Clock.now();

        // Add anything that needs to be run a single time when the OpMode starts
    }

    @Override
    public void loop() {
        // Allows CommandScheduler.run() to be called - DO NOT DELETE!
        super.loop();

        motorLeft.setPower(gamepad1.left_stick_y);
        motorRight.setPower(gamepad1.right_stick_y);

    }
}
