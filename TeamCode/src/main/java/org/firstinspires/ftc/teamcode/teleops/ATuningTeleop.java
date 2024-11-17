package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Pivot.kWristPos;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.used.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.used.PivotSubsystem;

@TeleOp
public class ATuningTeleop extends BaseRobot {

    private double debounce;
    @Override
    public void init(){
        super.init();
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)
                )
        );
//        pivot.setDefaultCommand(
//                new RunCommand(
//                        ()-> pivot.getWrist().setPosition(kWristPos)
//                )
//        );

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
        new Trigger(gamepad1.touchpad_finger_1, new RunCommand(()->{
            claw.changeClaw();
        }));

//        new Trigger(gamepad1.left_trigger>0.1, new RunCommand(()->{
//            intake.intaking(-gamepad1.left_trigger);
//        }));

        new Trigger(gamepad1.dpad_down, new RunCommand(()-> {
            pivot.decrementElbowDown();
        }));

        new Trigger(gamepad1.dpad_right, new RunCommand(() ->{
            pivot.incrementWristUp();
        }));
        new Trigger(gamepad1.dpad_left, new RunCommand(() ->{
            pivot.decrementWristDown();
        }));


        new Trigger(gamepad1.right_trigger > 0.1, new RunCommand(() -> {
            vertical.incrementSlide();
        }));
        new Trigger(gamepad1.left_trigger > 0.1, new RunCommand(() -> {
            vertical.decrementSlide();
        }));

        new Trigger(gamepad1.circle, new RunCommand(() -> {
            horizontal.incrementHorizSlide();
        }));
        new Trigger(gamepad1.square, new RunCommand(() -> {
            horizontal.decrementHorizSlide();
        }));
        new Trigger(gamepad1.triangle, new RunCommand(() -> {
            horizontal.incrementRail();
        }));
        new Trigger(gamepad1.cross, new RunCommand(() -> {
            horizontal.decrementRail();
        }));


        debounce = Clock.now();

    }



}
