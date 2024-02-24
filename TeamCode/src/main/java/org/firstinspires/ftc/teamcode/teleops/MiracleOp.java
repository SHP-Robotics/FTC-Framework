package org.firstinspires.ftc.teamcode.teleops;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MiracleBase;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

@TeleOp
public class MiracleOp extends MiracleBase {
    private double debounce;
    private double drivebias;
    // tee hee

    @Override
    public void init() {
        super.init();
        drivebias = 1.0;

        // Default command runs when no other commands are scheduled for the subsystem
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(-gamepad1.left_stick_y*drivebias, -gamepad1.left_stick_x*drivebias, -gamepad1.right_stick_x*drivebias)
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
            if (!Clock.hasElapsed(debounce, 0.5)) return;
            lift.incrementState();


            debounce = Clock.now();
        }));

        new Trigger(gamepad1.left_trigger > 0.5, new RunCommand(() -> {
            if (!Clock.hasElapsed(debounce, 0.5)) return;
            lift.deincrementState();
            debounce = Clock.now();
        }));

        new Trigger(gamepad1.dpad_up, new RunCommand(() -> {
            lift.goUp();
        }));

        new Trigger (gamepad1.dpad_down, new RunCommand(() -> {
            lift.goDown();
        }));

    }
}
