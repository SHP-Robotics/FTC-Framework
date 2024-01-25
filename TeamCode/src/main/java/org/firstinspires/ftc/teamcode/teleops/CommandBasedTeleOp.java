package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CRWheel;
import org.firstinspires.ftc.teamcode.subsystems.HookServo1;
import org.firstinspires.ftc.teamcode.subsystems.HookServo2;
import org.firstinspires.ftc.teamcode.subsystems.PixelServo;
import org.firstinspires.ftc.teamcode.subsystems.PlaneServo;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpinningIntake;

@TeleOp
public class CommandBasedTeleOp extends BaseRobot {
    private double debounce;

    @Override
    public void init() {
        super.init();


        // Default command runs when no other commands are scheduled for the subsystem
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x)
                )
        );
//        vision.setDefaultCommand(
//                new RunCommand(
//                        () -> vision.showRes(telemetry)
//                )
//        );

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
//
////        drive.setDriveBias(arm.getDriveBias());

        //////spinning intake
        new Trigger (gamepad2.a, new RunCommand(()->{
            spinningIntake.setState(SpinningIntake.State.IN);})
            .then(new RunCommand(()->{
                crWheel.setState(CRWheel.State.BACKWARD);
            }))
        );
        new Trigger (gamepad1.y, new RunCommand(()->{
            crWheel.setState(CRWheel.State.FORWARD);})

//                .then(new RunCommand(()->{
//                spinningIntake.setState(SpinningIntake.State.OUT);})
//                }))
        );
        new Trigger ((!gamepad1.a && !gamepad1.left_bumper && !gamepad1.y), new RunCommand(()->{
            spinningIntake.setState(SpinningIntake.State.DISABLE);
            })
                .then(new RunCommand(()->{
                    crWheel.setState(CRWheel.State.STILL);
                }))
        );
        //////right trigger
        new Trigger (gamepad1.right_trigger>0.5, new RunCommand(()->{
            adjustHolder.setState(WristSubsystem.State.UP);
        })
                .then(new RunCommand(()->{
                    arm.setState(ArmSubsystem.State.HIGH);

                }))
                .then(new RunCommand(()->{
                    slideServos.setState(ElbowSubsystem.State.UP);
                }))
//
        );
        new Trigger (gamepad1.left_trigger>0.5, new RunCommand(()->{
            adjustHolder.setState(WristSubsystem.State.DOWN);
        })
                .then(new RunCommand(()->{
                    slideServos.setState(ElbowSubsystem.State.DOWN);
                }))
                .then(new WaitCommand(1))
                .then(new RunCommand(()->{
                    arm.setState(ArmSubsystem.State.BOTTOM);
                }))
                .then(new RunCommand(()->{
                    pixelServo.setState(PixelServo.State.IN);
                }))

        );

        //drop 1
        new Trigger(gamepad1.right_bumper, new RunCommand(()->{
            pixelServo.setState(PixelServo.State.OUT);
        }));
        new Trigger(gamepad1.left_bumper, new RunCommand(()->{
            crWheel.setState(CRWheel.State.BACKWARD);
        }));

//
//        new Trigger (gamepad2.dpad_up, new RunCommand(()->{
//            crWheel.setState(CRWheel.State.FORWARD);
//        }));
//        new Trigger (gamepad2.dpad_down, new RunCommand(()->{
//            crWheel.setState(CRWheel.State.BACKWARD);
//        }));



        new Trigger (gamepad1.dpad_right, new RunCommand(()->{
            rightPlane.setState(HookServo1.State.ENGAGED); 
        })
                .then(new RunCommand(()->{
                    leftPlane.setState(HookServo2.State.ENGAGED);
                }))
//
        );

        new Trigger (gamepad1.dpad_left, new RunCommand(()->{
            planeServo.setState(PlaneServo.State.OUT);
        })
//
        );

        new Trigger(gamepad1.x, new RunCommand(()->{
            drive.incrementButtonClicks();
        }));

//        new Trigger (gamepad1.dpad_up, new RunCommand(()->{
//            arm.setState(ArmSubsystem.State.CLIMB);
//        })
////
//        );



//        new Trigger (gamepad1.dpad_right, new RunCommand(()->{
//            planeServo.setState(PlaneServo.State.IN);
//        })
//
//        );


//        new Trigger (gamepad1.b, new RunCommand(()->{
//            rightPlane.setState(HookServo1.State.DISENGAGED);
//            })
//                .then(new RunCommand(()->{
//                    leftPlane.setState(HookServo2.State.DISENGAGED);
//            }))
////
//        );
//        new Trigger (gamepad2.dpad_up, new RunCommand(()->{
//            slideServos.setState(PracticeArmServo.State.UP);
//        }));
//        new Trigger (gamepad2.dpad_down, new RunCommand(()->{
//            slideServos.setState(PracticeArmServo.State.DOWN);
//        }));
//
//        new Trigger (gamepad2.dpad_up, new RunCommand(()->{
//            slideServos.setState(PracticeArmServo.State.UP);
//        }));
//
//        new Trigger (gamepad1.a, new RunCommand(()->{
//            crWheel.setState(CRWheel.State.STILL);
//        }));
//        new Trigger (gamepad1.b, new RunCommand(()->{
//            crWheel.setState(CRWheel.State.FORWARD);
//        }));
//        new Trigger (gamepad1.x, new RunCommand(()->{
//            spinningIntake.setState(SpinningIntake.State.IN);
//        }));
//        new Trigger (gamepad1.y, new RunCommand(()->{
//            spinningIntake.setState(SpinningIntake.State.DISABLE);
//        }));
//        new Trigger (gamepad2.y, new RunCommand(()->{
//            pixelServo.setState(PixelServo.State.IN);
//        }));
//        new Trigger (gamepad2.a, new RunCommand(()->{
//            pixelServo.setState(PixelServo.State.OUT);
//        }));






















        /*
        new Trigger (gamepad1.a, new RunCommand(()->{
            if(spinningIntake.getState()==SpinningIntake.State.IN){
                spinningIntake.setState(SpinningIntake.State.DISABLE);
                CommandScheduler.getInstance().scheduleCommand(
                    new RunCommand(() -> {
                        crWheel.setState(CRWheel.State.BACKWARD);
                    }));

            }
            else{
                spinningIntake.setState(SpinningIntake.State.IN);
                  CommandScheduler.getInstance().scheduleCommand(
                    new RunCommand(() -> {
                        crWheel.setState(CRWheel.State.FORWARD);
                    })));
            }
        }));
         */

//        new Trigger(gamepad1.right_trigger>0.5, new RunCommand(()->{
//            arm.setState(ArmSubsystem.State.HIGH);
//        }));
//        new Trigger(gamepad1.left_trigger>0.5, new RunCommand(()->{
//            arm.setState(ArmSubsystem.State.BOTTOM);
//        }));



//        new Trigger(gamepad2.a, new RunCommand(()->{
//            slideServos.setState(PracticeArmServo.State.UP);
//        }));
//        new Trigger(gamepad2.b, new RunCommand(()->{
//            slideServos.setState(PracticeArmServo.State.DOWN);
//        }));

//        new Trigger(gamepad1.a, new RunCommand(() -> {
//            if (!Clock.hasElapsed(debounce, 0.5)) return;
//            if (arm.clawClosed()) {
//                arm.openClaw();
//                if (arm.atHub()) {
//                    arm.setState(ArmSubsystem.State.BOTTOM);
//                }
//            } else {
//                arm.closeClaw();
//                CommandScheduler.getInstance().scheduleCommand(
//                        new WaitCommand(0.5)
//                                .then(new RunCommand(() -> {
//                                    if (arm.atStacks()) arm.setState(ArmSubsystem.State.LOW);
//                                    else arm.setState(ArmSubsystem.State.HUB);
//                                }))
//                );
//            }
//            debounce = Clock.now();
//        }));
//        new Trigger(gamepad2.dpad_up, new RunCommand(()->{
//            spinningIntake.setState(SpinningIntake.State.IN);
//        }));
//        new Trigger(gamepad2.dpad_down, new RunCommand(()->{
//            spinningIntake.setState(SpinningIntake.State.OUT);
//        }));
//        new Trigger(gamepad2.a, new RunCommand(()->{
//            spinningIntake.setState(SpinningIntake.State.DISABLE);
//        }));
// Which buttons to map to? CRWheel
//        new Trigger(gamepad2.dpad_up, new RunCommand(()->{
//            cWheel.setState(CRWheel.State.FORWARD);
//        }));
//        new Trigger(gamepad2.dpad_down, new RunCommand(()->{
//            cWheel.setState(CRWheel.State.BACKWARD);
//        }));
//        new Trigger(gamepad2.a, new RunCommand(()->{
//            cWheel.setState(CRWheel.State.STILL);
//        }));

        // Which buttons to map to? Flap
//        new Trigger(gamepad2.b, new RunCommand(()->{
//            flap.setState(Flap.State.OPEN);
//        }));
//        new Trigger(gamepad2.y, new RunCommand(()->{
//            flap.setState(Flap.State.CLOSE);
//        }));
//        new Trigger(gamepad2.x, new RunCommand(()->{
//            flap.setState(Flap.State.NEUTRAL);
//        }));



//        new Trigger(gamepad1.a, new RunCommand(() -> {
//            if (!Clock.hasElapsed(debounce, 0.5)) return;
//            if (arm.clawClosed()) {
//                arm.openClaw();
//                if (arm.atHub()) {
//                    arm.setState(ArmSubsystem.State.BOTTOM);
//                }
//            } else {
//                arm.closeClaw();
//                CommandScheduler.getInstance().scheduleCommand(
//                        new WaitCommand(0.5)
//                                .then(new RunCommand(() -> {
//                                    if (arm.atStacks()) arm.setState(ArmSubsystem.State.LOW);
//                                    else arm.setState(ArmSubsystem.State.HUB);
//                                }))
//                );
//            }
//            debounce = Clock.now();
//        }));
//
//        new Trigger(gamepad1.b, new DropConeCommand(arm));
//
//        new Trigger(gamepad1.x, new RunCommand(() -> {
//            arm.setState(ArmSubsystem.State.STACK);
//        }));

//
//        new Trigger(gamepad1.dpad_right, new RunCommand(() -> {
//            arm.setState(ArmSubsystem.State.MIDDLE);
//        }));
//
//        new Trigger(gamepad1.dpad_left, new RunCommand(() -> {
//            arm.setState(ArmSubsystem.State.LOW);
//        }));
//
//        new Trigger(gamepad1.dpad_down, new RunCommand(() -> {
//            arm.setState(ArmSubsystem.State.BOTTOM);
//        }));
    }
}
