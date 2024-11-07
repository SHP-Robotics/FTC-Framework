package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Pivot.kWristPos;

import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class TuningTeleop extends BaseRobot {

    private double debounce;
    @Override
    public void init(){
        super.init();
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)
                )
        );
        pivot.setDefaultCommand(
                new RunCommand(
                        ()-> pivot.getWrist().setPosition(kWristPos)
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

//        new Trigger(gamepad1.right_bumper,
//                new RunCommand(
//                        () -> {pivot.getIntake().setState(IntakeSubsystem.State.INTAKING);
//                            pivot.getIntake().runServo();
//                            pivot.getWrist().setPosition(kWristPos);
//                        }
//                )
//        );
//        new Trigger(gamepad1.left_bumper,
//                new RunCommand(
//                        () -> {pivot.getIntake().setState(IntakeSubsystem.State.OUTAKING);
//                            pivot.getIntake().runServo();
//                            pivot.getWrist().setPosition(kWristPos);
//                        }
//                )
//        );
        new Trigger(gamepad1.dpad_up,
                new RunCommand(
                        () -> pivot.tuneElbowUp()
                )
        );

        new Trigger(gamepad1.dpad_down,
                new RunCommand(
                        ()-> pivot.tuneElbowDown()
                )
        );

        new Trigger(gamepad1.dpad_left,
                new RunCommand(
                        () -> pivot.tuneWristUp()
                )
        );
        new Trigger(gamepad1.dpad_right,
                new RunCommand(
                        () -> pivot.tuneWristDown()
                )
        );


        debounce = Clock.now();

    }



}
