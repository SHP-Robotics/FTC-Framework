package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Drive.kMaximumBias;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.commands.used.DrivetoSpecimenCommand;
import org.firstinspires.ftc.teamcode.commands.used.DrivetoSubCommand;
import org.firstinspires.ftc.teamcode.commands.used.DrivetoWallCommand;
import org.firstinspires.ftc.teamcode.commands.used.SpecimentoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.used.SubtoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.used.WalltoDriveCommand;
import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.used.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.used.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.used.RotateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.used.VerticalSubsystem;

@TeleOp
public class AOfficialTeleOp extends BaseRobot {
    private double driveBias;
    private boolean holdingRightBumper, holdingLeftBumper, holdingLeftTrigger;
    @Override
    public void init(){
        super.init();
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.newMecanum(driveBias*gamepad1.left_stick_y, driveBias*gamepad1.left_stick_x, driveBias*gamepad1.right_stick_x)
                )
        );
        holdingRightBumper = false;
        holdingLeftBumper = false;
        holdingLeftTrigger = false;
    }
    @Override
    public void start(){
        super.start();
        driveBias = vertical.getDriveBias();
    }

    @Override
    public void loop(){
        super.loop();
        driveBias = vertical.getDriveBias();

        //Vertical Slides
        new Trigger(gamepad1.dpad_up, new RunCommand(() -> {
            vertical.incrementSlide();

        }));
        new Trigger(gamepad1.dpad_down, new RunCommand(() -> {
            vertical.decrementSlide();
        }));

        //Intake from submersible
        new Trigger(gamepad1.right_bumper,
            new DrivetoSubCommand(rotate, claw, pivot, horizontal, gamepad1.right_trigger)
                    .then(new RunCommand(()->{
                        holdingRightBumper = true;
                    }))
        );
        new Trigger(holdingRightBumper && !gamepad1.right_bumper,
            new SubtoDriveCommand(rotate, claw, pivot, horizontal)
                    .then(new RunCommand(()->{
                        holdingRightBumper = false;
                    }))
                    .then(new WaitCommand(1))
                    .then(new RunCommand(()->{
                        rotate.setState(RotateSubsystem.State.NEUTRAL);
                        pivot.setState(PivotSubsystem.State.DRIVING);
                        horizontal.setState(HorizSubsystem.State.DRIVING);
                    }))
        );


        //intake specimen
        new Trigger(gamepad1.left_bumper,
            new DrivetoWallCommand(rotate, claw, pivot, horizontal)
                    .then(new RunCommand(()->{
                        holdingLeftBumper = true;
                    }))
        );
        new Trigger(holdingLeftBumper && !gamepad1.left_bumper,
            new WalltoDriveCommand(rotate, claw, pivot, horizontal)
                    .then(new RunCommand(()->{
                        holdingLeftBumper = false;
                    }))
                    .then(new WaitCommand(0.5))
                    .then(new RunCommand(()->{
                        pivot.setState(PivotSubsystem.State.DRIVING);
                    }))
        );

        //deposit specimen
        new Trigger(gamepad1.left_trigger > 0.5,
                new DrivetoSpecimenCommand(rotate, claw, pivot, horizontal, vertical)
                        .then(new RunCommand(()->{
                            holdingLeftTrigger = true;
                        }))
        );
        new Trigger(holdingLeftTrigger && gamepad1.left_trigger < 0.25,
                new SpecimentoDriveCommand(rotate, claw, pivot, horizontal, vertical)
                        .then(new RunCommand(()->{
                            holdingLeftTrigger = false;
                        }))
                        .then(new WaitCommand(0.5))
                        .then(new RunCommand(()->{
                            pivot.setState(PivotSubsystem.State.DRIVING);
                            vertical.setState(VerticalSubsystem.State.BOTTOM);
                        }))
        );

        //deposit sample
//        new Trigger(gamepad1.left_trigger > 0.5, new RunCommand(()->{
//            //if driving deposit, if deposit, drive
//        }));

        //Claw
        new Trigger(gamepad1.left_bumper && gamepad1.right_bumper, new RunCommand(() -> {
            if(pivot.getState()== PivotSubsystem.State.PREPAREINTAKE)
                rotate.rotateCW();
        }));
        new Trigger(gamepad1.left_trigger>0.25 && gamepad1.right_bumper, new RunCommand(()->{
            if(pivot.getState()== PivotSubsystem.State.PREPAREINTAKE)
                rotate.rotateCCW();
        }));

        //resetIMU
        new Trigger(gamepad1.triangle, new RunCommand(()->{
            drive.resetIMUAngle();
        }));


        //reset Slide Zero Pos
        new Trigger(gamepad2.square, new RunCommand(()->{
            vertical.resetZeroPosition();
        }));

        //toggle high or low bars
        new Trigger(gamepad2.triangle, new RunCommand(()->{
            vertical.cycleStates(true); //FOR BAR
        }));
        new Trigger(gamepad2.circle, new RunCommand(()->{
            vertical.cycleStates(false); //FOR BUCKET
        }));

    }


}
