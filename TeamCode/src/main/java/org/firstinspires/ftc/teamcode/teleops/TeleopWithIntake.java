package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp
public class TeleopWithIntake extends BaseRobot {
    private double debounce;
    @Override
    public void init(){
        super.init();
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)
                )
        );

    }
    @Override
    public void start(){
        super.start();
        debounce = Clock.now();
    }

    @Override
    public void loop(){
        super.loop();
        new Trigger(gamepad1.right_bumper,
                new RunCommand(
                        () -> {intake.setState(IntakeSubsystem.State.INTAKING);
                            intake.runServo();
                        }
                )
        );
        new Trigger(gamepad1.left_bumper,
                new RunCommand(
                        () -> {intake.setState(IntakeSubsystem.State.OUTAKING);
                            intake.runServo();
                        }
                )
        );


    }


}
