package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.devices.GamepadKey;

import org.firstinspires.ftc.teamcode.commands.BuckettoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DrivetoBucketCommand;
import org.firstinspires.ftc.teamcode.commands.DrivetoHumanCommand;
import org.firstinspires.ftc.teamcode.commands.DrivetoSpecimenCommand;
import org.firstinspires.ftc.teamcode.commands.DrivetoSubCommand;
import org.firstinspires.ftc.teamcode.commands.DrivetoWallCommand;
import org.firstinspires.ftc.teamcode.commands.HumantoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.SpecimentoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.SubtoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.WalltoDriveCommand;
import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSubsystem;

@TeleOp
public class AOfficialTeleOp extends BaseRobot {
    private double driveBias;
    private boolean holdingRightBumper, LBTrigger, LTTrigger, crossTrigger;

    GamepadInterface gamepadInterface1, gamepadInterface2;

    @Override
    public void init(){
        super.init();
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(-driveBias*gamepad1.left_stick_y, driveBias*gamepad1.left_stick_x, driveBias*gamepad1.right_stick_x)
                )
        );
        holdingRightBumper = false;
        LBTrigger = true;
        LTTrigger = true;
        crossTrigger = true;

        gamepadInterface1 = new GamepadInterface(gamepad1);
        gamepadInterface2 = new GamepadInterface(gamepad2);

    }
    @Override
    public void start(){
        super.start();
        driveBias = vertical.getDriveBias();

        //pivot.setState(PivotSubsystem.State.DRIVING);
    }

    @Override
    public void loop(){
        super.loop();
        driveBias = vertical.getDriveBias();
        gamepadInterface1.update();
        drive.update(gamepad1);

        gamepadInterface2.update();
        drive.update(gamepad2);
//        drive.drive.setZeroPowerBehavior(gamepad2.cross ? BRAKE : FLOAT);
        // use gamepad2.right_bumper as speed boost

        //Intake from submersible
        new Trigger(gamepad1.right_bumper,
            new DrivetoSubCommand(rotate, claw, pivot, horizontal, gamepad1.right_trigger)
                    .then(new RunCommand(()->{
                        holdingRightBumper = true;
                    }))
        );
        new Trigger((holdingRightBumper && !gamepad1.right_bumper),
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
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.LEFT_BUMPER) && LBTrigger,
                new DrivetoWallCommand(rotate, claw, pivot, horizontal)
                        .then(new RunCommand(()->{
                            LBTrigger = false;
                        }))
        );

        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.LEFT_BUMPER) && !LBTrigger,
                new WalltoDriveCommand(rotate, claw, pivot, horizontal)
                        .then(new RunCommand(()->{
                            LBTrigger = true;
                        }))
                        .then(new WaitCommand(0.5))
                        .then(new RunCommand(()->{
                            pivot.setState(PivotSubsystem.State.DRIVING);
                        }))
        );

        //deposit specimen
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.LEFT_TRIGGER) && LTTrigger,
                new DrivetoSpecimenCommand(rotate, claw, pivot, horizontal, vertical)
                .then(new RunCommand(()->{
                    LTTrigger = false;
                }))
        );
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.LEFT_TRIGGER) && !LTTrigger,
                new SpecimentoDriveCommand(rotate, claw, pivot, horizontal, vertical)
                .then(new RunCommand(()->{
                    LTTrigger = true;
                }))
                .then(new WaitCommand(0.5))
                .then(new RunCommand(()->{
                    pivot.setState(PivotSubsystem.State.PREPAREDRIVING);
                    claw.close();
                }))
                .then(new WaitCommand(0.5))
                .then(new RunCommand(()->{
                    pivot.setState(PivotSubsystem.State.DRIVING);
                    vertical.setState(VerticalSubsystem.State.BOTTOM);
                }))
        );

        //deposit bucket
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.A) && crossTrigger,
                new DrivetoBucketCommand(rotate, claw, pivot, horizontal, vertical)
                .then(new RunCommand(()->{
                    crossTrigger = false;
                }))

        );
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.A) && !crossTrigger,
                new BuckettoDriveCommand(rotate, claw, pivot, horizontal, vertical)
                        .then(new RunCommand(()->{
                            crossTrigger = true;
                        }))
                        .then(new WaitCommand(0.5))
                        .then(new RunCommand(()->{
                            pivot.setState(PivotSubsystem.State.DRIVING);
                            vertical.setState(VerticalSubsystem.State.BOTTOM);
                        }))

        );

        //Claw
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.DPAD_LEFT), new RunCommand(() -> {
                rotate.rotateCW();
        }));
        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.DPAD_RIGHT), new RunCommand(()->{
                rotate.rotateCCW();
        }));

        //give sample to human player
        new Trigger(gamepad1.square, new DrivetoHumanCommand(rotate, claw, pivot, horizontal)
                .then(new WaitCommand(0.75))
                .then(new HumantoDriveCommand(rotate, claw, pivot, horizontal)));


        //resetIMU
        new Trigger(gamepad1.triangle, new RunCommand(()->{
            drive.resetIMUAngle();
        }));


        //reset Slide Zero Pos
        new Trigger(gamepad2.square, new RunCommand(()->{
            vertical.resetZeroPosition();
        }));

        //Reset encoders
        new Trigger(gamepadInterface2.isKeyDown(GamepadKey.DPAD_UP), new RunCommand(() -> {
            vertical.incrementSlide();
        }));
        new Trigger(gamepadInterface2.isKeyDown(GamepadKey.DPAD_DOWN), new RunCommand(()->{
            vertical.emergencyDecrementSlide();
        }));

        new Trigger(gamepadInterface2.isKeyDown(GamepadKey.DPAD_LEFT), new RunCommand(()->{
            vertical.endReset();
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
