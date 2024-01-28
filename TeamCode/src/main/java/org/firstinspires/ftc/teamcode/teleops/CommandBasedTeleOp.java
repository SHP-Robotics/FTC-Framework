package org.firstinspires.ftc.teamcode.teleops;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.debug.OneMotorSystem;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PlaneSubsystem;

@TeleOp
public class CommandBasedTeleOp extends BaseRobot {
    private double debounce;
    private double drivebias;
    // tee hee
    private OneMotorSystem elbow;
    private Servo wrist;
    private Servo plane;

    @Override
    public void init() {
        super.init();
        drivebias = 1.0;
        claw.close();
        arm.setState(ArmSubsystem.State.DRIVE);

        // Default command runs when no other commands are scheduled for the subsystem
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(-gamepad1.left_stick_y*drivebias, -gamepad1.left_stick_x*drivebias, -gamepad1.right_stick_x*drivebias)
                )
        );

        // tee hee
        plane = hardwareMap.get(Servo.class, "plane");

        elbow = new OneMotorSystem.OneMotorSystemBuilder(hardwareMap, "elbow")
                .setDirection(DcMotorSimple.Direction.FORWARD)
                .setUseBrakes(false)
                .setUseEncoders(true)
                .setStaticPower(0)
                .build();

        elbow.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wrist = hardwareMap.get(Servo.class, "wrist");
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

        // tee hee
        // sus :|
//        drive.setDriveBias(arm.getDriveBias());

        //TODO: Open/close macro to circle DONE
        new Trigger(gamepad1.circle, new RunCommand(()->{
            if (!Clock.hasElapsed(debounce, 0.5)) return;

            claw.nextState();
            debounce = Clock.now();
        }));

        new Trigger(gamepad1.right_trigger>0.5, new RunCommand(()->{ //all upward stuff
            if (!Clock.hasElapsed(debounce, 0.5)) return;

            if((claw.closed() && arm.getState() == ArmSubsystem.State.INTAKE)
            || arm.getState() == ArmSubsystem.State.MANUAL
                    || arm.getState() == ArmSubsystem.State.UNKNOWN){
                claw.close();

                //tee hee
//                arm.setState(ArmSubsystem.State.DRIVE);

                elbow.setPosition((int)Constants.Arm.kElbowDrive, false);
                wrist.setPosition((int)Constants.Arm.kWristDrive);

                drivebias = 1.0;
            }
            else if(arm.getState() == ArmSubsystem.State.DRIVE){
                arm.setState(ArmSubsystem.State.OUTTAKE);
                drivebias = 0.75;
            }



            debounce = Clock.now();
        }));
        new Trigger(gamepad1.left_trigger>0.5, new RunCommand(()->{ //all downward stuff
            if (!Clock.hasElapsed(debounce, 0.5)) return;

            if(arm.getState() == ArmSubsystem.State.OUTTAKE
                    || arm.getState() == ArmSubsystem.State.MANUAL
                    || arm.getState() == ArmSubsystem.State.UNKNOWN){
                claw.close();
                drivebias = 1.0;
                arm.setState(ArmSubsystem.State.DRIVE);

                // tee hee
                elbow.setPosition((int)Constants.Arm.kElbowDown, false);
            }
            else if(arm.getState() == ArmSubsystem.State.DRIVE){
                arm.setState(ArmSubsystem.State.INTAKE);
                drivebias = 0.75;
                claw.open();
            }

            debounce = Clock.now();
        }));

        new Trigger(gamepad1.dpad_up, new RunCommand(() -> {
            // tee hee
            elbow.setLiftPower(0.5);

//            arm.setState(ArmSubsystem.State.MANUAL);
//
//            arm.upElbow();
        }));
        new Trigger(gamepad1.dpad_down, new RunCommand(() -> {
            // tee hee
            elbow.setLiftPower(-0.5);

//            arm.setState(ArmSubsystem.State.MANUAL);
//
//            arm.downElbow();
        }));

        //TODO: PLANE LAUNCH BOTH BUMPERS
        // tehe
        if (gamepad1.left_bumper) {
            plane.setPosition(1);
        }
//        new Trigger(gamepad1.left_bumper, new RunCommand(() -> {
//                plane.setState(PlaneSubsystem.State.LAUNCH);
//        }));

        new Trigger(gamepad1.square, new RunCommand(() -> {
            drive.resetIMUAngle();
        }));

        //TODO: reset heading dpad up
        //TODO: fast speed when in drive mode
        //TODO: slow speed when in intake/outtake mode

//        new Trigger(gamepad1.dpad_up, new RunCommand(() -> {
//            lift.setState(LiftSubsystem.State.PREPARE);
//        }));
//        new Trigger(gamepad1.dpad_up, new RunCommand(() -> {
//            lift.setState(LiftSubsystem.State.CLIMB);
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

//        new Trigger(gamepad1.b, new DropConeCommand(arm));
//
//        new Trigger(gamepad1.x, new RunCommand(() -> {
//            arm.setState(ArmSubsystem.State.STACK);
//        }));
//
//        new Trigger(gamepad1.dpad_up, new RunCommand(() -> {
//            arm.setState(ArmSubsystem.State.HIGH);
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
