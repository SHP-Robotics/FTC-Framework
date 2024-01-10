package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.Constants.offset;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.TestBaseRobot;
import org.firstinspires.ftc.teamcode.commands.LowerArmCommand;
import org.firstinspires.ftc.teamcode.commands.RaiseArmCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Flap;
import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PixelServo;
import org.firstinspires.ftc.teamcode.subsystems.PlaneServo;

@TeleOp
public class ATestTeleop extends TestBaseRobot {
    private double debounce;
    private double driveBias;

    @Override
    public void init() {
        super.init();
        intake.dropDown.setPosition(0.25);

        // Default command runs when no other commands are scheduled for the subsystem
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(-gamepad1.left_stick_y*driveBias, -gamepad1.left_stick_x*driveBias, -gamepad1.right_stick_x*driveBias)
                )
        );
//        vision.setDefaultCommand(
//                new RunCommand(
//                        () -> vision.showRes(telemetry)
//                )
//        );
        driveBias = 1;
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
        ///testing intack for stack
        new Trigger(gamepad2.a,
            new RunCommand(()->{
                arm.setState(ArmSubsystem.State.CONESTACK);
            })
        );


        //////spinning intake
        new Trigger (gamepad1.right_bumper,
                new RaiseArmCommand(arm,wrist,elbow,pixelServo)
        );
        new Trigger(gamepad1.square, new RunCommand(() -> {
            if (!Clock.hasElapsed(debounce, 0.1)) return;
            offset -= 0.002;
        }));
        //good
        new Trigger(gamepad1.left_trigger > 0.5,new RunCommand(()->{
            if (!Clock.hasElapsed(debounce, 0.5)) return;
            driveBias = 0.5;
        })
        );
        //good
        new Trigger(gamepad1.left_trigger < 0.5,new RunCommand(()->{
            driveBias = 1;
        })
        );
        new Trigger (gamepad1.left_bumper,
                new LowerArmCommand(arm,wrist,elbow)
                        .then(new RunCommand(()->{
                            pixelServo.setState(PixelServo.State.IN);
                        }))
        );
        new Trigger ((!gamepad1.a && !gamepad1.y), new RunCommand(()->{
            intake.setState(IntakeSubsystem.State.STILL);
        })
        );

        new Trigger (gamepad1.y, new RunCommand(()->{
            intake.setState(IntakeSubsystem.State.REJECT);
            offset = 0;
        })
        );
        new Trigger (gamepad1.b,new RunCommand(()->{
            drive.resetIMUAngle();
        })
        );

        new Trigger (gamepad1.a, new RunCommand(()->{
            if (arm.getState() == ArmSubsystem.State.BOTTOM)       //1. if arm is at bottom
                intake.setState(IntakeSubsystem.State.INTAKING);   //   intake pixels
            else if (pixelServo.getState() == PixelServo.State.IN) //2. if no pixels have been released
                pixelServo.setState(PixelServo.State.OUT);         //   release pixel #1
            else                                                   //3. if pixel #1 has been released
                intake.setState(IntakeSubsystem.State.OUTTAKING);  //   release pixel #2
            })
        );

        new Trigger (gamepad1.dpad_left, new RunCommand(()->{
            planeServo.setState(PlaneServo.State.OUT);
        })
        );

        new Trigger (gamepad1.dpad_up, new RunCommand(()->{
            arm.setState(ArmSubsystem.State.CLIMB);
        })
        );

        new Trigger (gamepad1.dpad_down,
            new RunCommand(()->{
                arm.setState(ArmSubsystem.State.BOTTOMCLIMB);
            })
            .then(new WaitCommand(2))
            .then(new RunCommand(()->{
                hook.setState(HookSubsystem.State.ENGAGED);
            }))
            .then(new WaitCommand(0.5))
            .then(new RunCommand(()->{
                arm.setState(ArmSubsystem.State.FINISHCLIMB);
            }))
        );

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
