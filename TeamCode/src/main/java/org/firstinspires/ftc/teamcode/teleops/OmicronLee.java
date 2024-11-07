package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

@TeleOp
public class OmicronLee extends BaseRobot {
    private double debounce;
    private double driveBias;
    private int pixels = 0;

    @Override
    public void init() {
        super.init();

        // Default command runs when no other commands are scheduled for the subsystem
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(-gamepad1.left_stick_y*driveBias, -gamepad1.left_stick_x*driveBias, -gamepad1.right_stick_x*driveBias)
                )
        );

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
        telemetry.update();

        new Trigger(gamepad1.dpad_up, new RunCommand(()->{
            arm.incrementSlide();
        }));
        new Trigger(gamepad1.dpad_down, new RunCommand(()->{
            arm.decrementSlide();
        }));
        new Trigger(gamepad1.left_bumper, new RunCommand(()->{
            arm.incrementRotate();
        }));
        new Trigger(gamepad1.right_bumper, new RunCommand(()->{
            arm.decrementRotate();
        }));
        



//        // Boost button
//        new Trigger(gamepad1.left_stick_button, new RunCommand(() -> {
//            if (!Clock.hasElapsed(debounce, 0.5)) return;
//            if(driveBias == 0.3)
//                driveBias = 1.0;
//            else
//                driveBias = 0.3;
//        }));

//        new Trigger(gamepad1.right_stick_button && !holdingRightStick, new RunCommand(() -> {
//             if state ordinal = 0 -> ordinal = 1
//             if state ordinal = 1 -> ordinal = 2 -> ordinal = 0 because of modulo
//            dropDown.setState(DropDownSubsystem.State.values()[(dropDown.getState().ordinal()+1)%6]);
//        }));

        // Lower arm
//        new Trigger((arm.getState() != ArmSubsystem.State.BOTTOM && gamepad1.left_bumper) || (wrist.getState() == WristSubsystem.State.STAGE_DOOR && !gamepad1.touchpad),
//                new LowerArmCommand(arm, wrist, elbow)
//        );
//
//        // Set intake to still
//        new Trigger((gamepad1.right_trigger<0.5 && gamepad1.left_trigger<0.5&&!gamepad1.triangle), new RunCommand(() -> {
//            if(intake.getState() != IntakeSubsystem.State.DEPOSIT1)
//                intake.setState(IntakeSubsystem.State.STILL);
//            if (dropdownTimer.seconds() > 1.5) {
//                dropDown.setState(DropDownSubsystem.State.RAISED);
//            }
//        }));
//
//        // Set intake to intake
//        new Trigger((gamepad1.right_trigger>0.5 && arm.getState() == ArmSubsystem.State.BOTTOM), new RunCommand(() -> {
//            if(arm.getSlidePosition()<20)
//                intake.setState(IntakeSubsystem.State.INTAKE);
//        }));
//        new Trigger((gamepad1.right_trigger>0.5 && arm.getSlidePosition() < Constants.Arm.kSlideTolerance), new RunCommand(() -> intake.setState(IntakeSubsystem.State.INTAKE)));
//        new Trigger((gamepad1.right_trigger>0.5 && arm.getSlidePosition() < Constants.Arm.kSlideTolerance), new RunCommand(() -> {
//            intake.setState(IntakeSubsystem.State.INTAKE);
//            dropDown.setState(DropDownSubsystem.State.GROUND_HEIGHT);
//            dropdownTimer.reset();
//        }));
//
//        // Set intake to reject
//        new Trigger((gamepad1.left_trigger > 0.5 && arm.getState() == ArmSubsystem.State.BOTTOM), new RunCommand(() -> intake.setState(IntakeSubsystem.State.REJECT)));
//
//        new Trigger((gamepad1.dpad_right && arm.getState() == ArmSubsystem.State.BOTTOM), new RunCommand(() -> intake.setState(IntakeSubsystem.State.REJECT_ALL)));
//
//        // wait before depositing more pixels
//        new Trigger(gamepad1.triangle && !holdingTriangle, new RunCommand(() -> {
//            if (intake.getState() == IntakeSubsystem.State.STILL) {                         //1. if no pixels have been released
//                intake.setState(IntakeSubsystem.State.DEPOSIT1);                            //   release pixel #1
//                telemetry.addData("Pixels Deposited: ", 1);
//            } else {                                                                        //3. if pixel #1 has been released
//                intake.setState(IntakeSubsystem.State.DEPOSIT2);                            //   release pixel #2
//                telemetry.addData("Pixels Deposited: ", 2);
//            }
//        }));
//
//        holdingTriangle = gamepad1.triangle;
//
//        // Reset IMU
//        new Trigger(gamepad1.square, new RunCommand(() -> drive.resetIMUAngle()));
//
//        // new code from WillyBilly
//        // allows spamming
//
//        // move arm up
//        if (gamepad1.right_bumper) {
//            if (!holdingRightBumper) {
//                new Trigger(true,
//                        new IncrementUpArmCommand(arm,wrist,elbow)
//                );
//            }
//            holdingRightBumper = true;
//        } else {
//            holdingRightBumper = false;
//        }
//
//        // new code from WillyBilly
//        // allows spamming
//
//        // move arm down
//        if (gamepad1.cross) {
//            if (!holdingCross) {
//                new Trigger(true,
//                        new DecrementDownArmCommand(arm,wrist,elbow)
//                );
//            }
//            holdingCross = true;
//        } else {
//            holdingCross = false;
//        }
//        if(sensor.filled()) {
//            if (pixels != 2){
//                gamepad1.rumble(1000);
//                pixels = 2;
//            }
//        }
//        else{
//            pixels = 0;
//        }
//        // launch plane
//        new Trigger (gamepad1.dpad_left, new RunCommand(()->{
////            if (!Clock.hasElapsed(debounce, 60)) return;
//            planeServo.setState(PlaneServo.State.OUT);
//        }));
//
//        // prepare climb
//        new Trigger (gamepad1.dpad_up, new PrepareClimbCommand(arm, wrist, elbow));
//
//        // finish climb
//        new Trigger (gamepad1.dpad_down && arm.getState() == ArmSubsystem.State.CLIMB, new RunCommand(()->arm.setState(ArmSubsystem.State.FINISHCLIMB)));
//
//        // open door
//        new Trigger (gamepad1.touchpad && arm.getState() == ArmSubsystem.State.BOTTOM, new OpenStageDoorCommand(arm, wrist, elbow));
//
//        new Trigger(gamepad1.touchpad && !holdingTouchpad && wrist.getState() == WristSubsystem.State.MOSAIC, new RunCommand(() -> {
//            elbow.setState(ElbowSubsystem.State.UP);
//            wrist.setState(WristSubsystem.State.UP);
//        }));
//
//        // rearrange mosaics
//        new Trigger(gamepad1.touchpad && !holdingTouchpad && arm.getState() != ArmSubsystem.State.BOTTOM, new RearrangeMosaicCommand(arm, wrist, elbow));
//
//        holdingTouchpad = gamepad1.touchpad;
    }
}
