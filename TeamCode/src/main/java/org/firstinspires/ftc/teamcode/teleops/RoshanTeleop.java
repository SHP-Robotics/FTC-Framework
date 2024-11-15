package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

@TeleOp
public class RoshanTeleop extends BaseRobot {
    private double debounce;
    private double driveBias;

    @Override
    public void init(){
        super.init();
        driveBias = 1;

        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(-gamepad1.left_stick_y*driveBias, -gamepad1.left_stick_x*driveBias, -gamepad1.right_stick_x*driveBias)
                )
        );
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
        telemetry.update();

        new Trigger(gamepad1.right_trigger > 0.5, new RunCommand(() -> {
            intake.getServo().setPower(1.0);
        }));
        new Trigger(gamepad1.left_trigger > 0.5, new RunCommand(() -> {
            intake.getServo().setPower(-1.0);
        }));


    }

}
