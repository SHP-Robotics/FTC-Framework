package org.firstinspires.ftc.teamcode.teleops;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Const;
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

    private Servo climb;

    @Override
    public void init() {
        super.init();
        drivebias = 1.0;
        claw.close();
        arm.setState(ArmSubsystem.State.INTAKE);

        // Default command runs when no other commands are scheduled for the subsystem
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(-gamepad1.left_stick_y*drivebias, -gamepad1.left_stick_x*drivebias, -gamepad1.right_stick_x*drivebias)
                )
        );

//        // tee hee
//        plane = hardwareMap.get(Servo.class, "plane");

        plane = hardwareMap.get(Servo.class, "plane");
        plane.setPosition(0.6);

        elbow = new OneMotorSystem.OneMotorSystemBuilder(hardwareMap, "elbow")
                .setDirection(DcMotorSimple.Direction.REVERSE)
                .setUseBrakes(false)
                .setUseEncoders(true)
                .setStaticPower(0)
                .build();

        elbow.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(Constants.Arm.kWristDrive);

        climb = hardwareMap.get(Servo.class, "climb");
        climb.setPosition(Constants.Climb.kClimbHold);

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
        telemetry.addData("Arm state: ", arm.getState());
        telemetry.addData("Arm pos: ", elbow.getMotorPosition());
        telemetry.update();

        // tee hee
        // sus :|
//        return Math.abs(getElbowPosition(MotorUnit.TICKS) / kElbowUp - 1.0);

        drive.setDriveBias(Math.abs(elbow.getMotorPosition() / Constants.Arm.kElbowUp - 1.0));

        elbow.update((int)Constants.Arm.kElbowTolerance);

        //TODO: Open/close macro to circle DONE

        new Trigger(gamepad1.right_bumper, new RunCommand(() -> {
            climb.setPosition(Constants.Climb.kClimbRelease);
        }));

        new Trigger(gamepad1.circle, new RunCommand(()->{
            if (!Clock.hasElapsed(debounce, 0.5)) return;

            claw.nextState();
            debounce = Clock.now();
        }));

        new Trigger(gamepad1.right_trigger>0.5, new RunCommand(()->{ //all upward stuff
            if (!Clock.hasElapsed(debounce, 0.5)) return;
            if (!claw.closed() && arm.getState() == ArmSubsystem.State.INTAKE) {
                claw.close();
            }
            else if((claw.closed() && arm.getState() == ArmSubsystem.State.INTAKE)
            || arm.getState() == ArmSubsystem.State.MANUAL
                    || arm.getState() == ArmSubsystem.State.UNKNOWN){
//                claw.close(); fix shit not opening?

//                tee hee
                arm.setState(ArmSubsystem.State.DRIVE);

                elbow.setPosition((int)Constants.Arm.kElbowDrive, false, (int)Constants.Arm.kElbowTolerance);
                wrist.setPosition(Constants.Arm.kWristDrive);

                drivebias = 1.0;
            }
            else if(arm.getState() == ArmSubsystem.State.DRIVE){
                arm.setState(ArmSubsystem.State.OUTTAKE);

                elbow.setPosition((int)Constants.Arm.kElbowUp, false, (int)Constants.Arm.kElbowTolerance);
                wrist.setPosition(Constants.Arm.kWristDeposit);

                drivebias = 0.75;
            }
            else if (arm.getState() == ArmSubsystem.State.OUTTAKE){
                arm.setState(ArmSubsystem.State.REACH);
                elbow.setPosition((int)Constants.Arm.kElbowReach, false, (int)Constants.Arm.kElbowTolerance);
                wrist.setPosition(Constants.Arm.kWristReach);
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
                wrist.setPosition(Constants.Arm.kWristDrive);

                // tee hee
                elbow.setPosition((int)Constants.Arm.kElbowDrive, false, (int)Constants.Arm.kElbowTolerance);
            }
            else if(arm.getState() == ArmSubsystem.State.DRIVE){
                arm.setState(ArmSubsystem.State.INTAKE);
                elbow.setPosition((int)Constants.Arm.kElbowDown, false, (int)Constants.Arm.kElbowTolerance);
                wrist.setPosition(Constants.Arm.kWristDown);
                drivebias = 0.75;
//                claw.open();
            }
            else if (arm.getState() == ArmSubsystem.State.INTAKE) {
                claw.open();
                wrist.setPosition(Constants.Arm.kWristDown);
            }
            else if (arm.getState() == ArmSubsystem.State.CLIMB) {
                arm.setState(ArmSubsystem.State.INTAKE);
                wrist.setPosition(Constants.Arm.kWristDown);
                elbow.setPosition((int) Constants.Arm.kElbowDown, false, (int) Constants.Arm.kElbowTolerance);

            }
            else if (arm.getState() == ArmSubsystem.State.REACH) {
                arm.setState(ArmSubsystem.State.OUTTAKE);
                elbow.setPosition((int)Constants.Arm.kElbowUp, false, (int)Constants.Arm.kElbowTolerance);
                wrist.setPosition(Constants.Arm.kWristDeposit);
            }


            debounce = Clock.now();
        }));

        new Trigger(gamepad1.dpad_up, new RunCommand(() -> {
            // tee hee
            arm.setState(ArmSubsystem.State.MANUAL);

            elbow.manual(1, 100);

//
//            arm.upElbow();
        }));
        new Trigger(gamepad1.dpad_down, new RunCommand(() -> {
            // tee hee
            arm.setState(ArmSubsystem.State.MANUAL);

            elbow.manual(1, -100);

//            arm.setState(ArmSubsystem.State.MANUAL);
//
//            arm.downElbow();
        }));

        new Trigger(gamepad1.dpad_right, new RunCommand(() -> {
            wrist.setPosition(0.1);
            elbow.setPosition((int) Constants.Arm.kElbowClimb, false, (int) Constants.Arm.kElbowTolerance);
            arm.setState(ArmSubsystem.State.CLIMB);
        }));

//        new Trigger(!(gamepad1.dpad_down || gamepad1.dpad_up), new RunCommand(() -> {
//            elbow.applyBrakes();
//        }));

        //TODO: PLANE LAUNCH BOTH BUMPERS
        // tehe
        if (gamepad1.left_bumper) {
            plane.setPosition(1.0);
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
