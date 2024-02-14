package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.DecrementDownArmCommand;
import org.firstinspires.ftc.teamcode.commands.IncrementUpArmCommand;
import org.firstinspires.ftc.teamcode.commands.LowerArmCommand;
import org.firstinspires.ftc.teamcode.commands.OpenStageDoorCommand;
import org.firstinspires.ftc.teamcode.commands.PrepareClimbCommand;
import org.firstinspires.ftc.teamcode.shplib.Constants;
import org.firstinspires.ftc.teamcode.shplib.TestBaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PlaneServo;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@TeleOp
public class AndyRandy extends TestBaseRobot {
    private double debounce;
    private double driveBias;

    // spammable code
    private boolean holdingCross = false;
    private boolean holdingRightBumper = false;

    private ElapsedTime elapsedTime;
    private boolean holdingTriangle = false;

    @Override
    public void init() {
        super.init();

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

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

        drive.setDriveBias(arm.getDriveBias());

        // Boost button
        new Trigger(gamepad1.left_stick_button, new RunCommand(() -> {
            if (!Clock.hasElapsed(debounce, 0.5)) return;
            if(driveBias == 0.3)
                driveBias = 1.0;
            else
                driveBias = 0.3;
        }));

        // Lower arm
        new Trigger((arm.getState() != ArmSubsystem.State.BOTTOM && gamepad1.left_bumper) || (wrist.getState() == WristSubsystem.State.STAGE_DOOR && !gamepad1.touchpad),
            new LowerArmCommand(arm, wrist, elbow)
        );

        // Set intake to still
        new Trigger((gamepad1.right_trigger<0.5 && gamepad1.left_trigger<0.5&&!gamepad1.triangle), new RunCommand(() -> {
            if(intake.getState() != IntakeSubsystem.State.DEPOSIT1)
                intake.setState(IntakeSubsystem.State.STILL);
        }));

        // Set intake to intake
        new Trigger((gamepad1.right_trigger>0.5 && arm.getSlidePosition() < Constants.Arm.kSlideTolerance), new RunCommand(() -> intake.setState(IntakeSubsystem.State.INTAKE)));

        // Set intake to reject
        new Trigger((gamepad1.left_trigger > 0.5 && arm.getState() == ArmSubsystem.State.BOTTOM), new RunCommand(() -> intake.setState(IntakeSubsystem.State.REJECT)));

        new Trigger((gamepad1.dpad_right && arm.getState() == ArmSubsystem.State.BOTTOM), new RunCommand(() -> intake.setState(IntakeSubsystem.State.REJECTALL)));

        // wait before depositing more pixels
        if (gamepad1.triangle) {
            if (!holdingTriangle) {
                elapsedTime.reset();
            } else if (elapsedTime.seconds() > 0.04) {
                // Deposit variable pixels
                new Trigger(gamepad1.triangle, new RunCommand(() -> {
                    if (intake.getState() != IntakeSubsystem.State.STILL) { //2. if no pixels have been released
                        intake.setState(IntakeSubsystem.State.DEPOSIT2);       //   release pixel #1
                        holdingTriangle = false;
                        telemetry.addData("Pixels Deposited: ", 2);
                    } else {                                                //3. if pixel #1 has been released
                        intake.setState(IntakeSubsystem.State.DEPOSIT1);  //   release pixel #2
                        telemetry.addData("Pixels Deposited: ", 1);
                    }
                }));
            }
            holdingTriangle = true;
        } else {
            holdingTriangle = false;
        }

        // Reset IMU
        new Trigger(gamepad1.square, new RunCommand(() -> drive.resetIMUAngle()));

        // new code from WillyBilly
        // allows spamming

        // move arm up
        if (gamepad1.right_bumper) {
            if (!holdingRightBumper) {
                new Trigger(true,
                        new IncrementUpArmCommand(arm,wrist,elbow)
                );
            }
            holdingRightBumper = true;
        } else {
            holdingRightBumper = false;
        }

        // new code from WillyBilly
        // allows spamming

        // move arm down
        if (gamepad1.cross) {
            if (!holdingCross) {
                new Trigger(true,
                        new DecrementDownArmCommand(arm,wrist,elbow)
                );
            }
            holdingCross = true;
        } else {
            holdingCross = false;
        }

        // launch plane
        new Trigger (gamepad1.dpad_left, new RunCommand(()->{
            if (!Clock.hasElapsed(debounce, 60)) return;
            planeServo.setState(PlaneServo.State.OUT);
        }));

        // prepare climb
        new Trigger (gamepad1.dpad_up, new PrepareClimbCommand(arm, wrist, elbow));

        // finish climb
        new Trigger (gamepad1.dpad_down && arm.getState() == ArmSubsystem.State.CLIMB, new RunCommand(()->arm.setState(ArmSubsystem.State.FINISHCLIMB)));

        // open door
        new Trigger (gamepad1.touchpad, new OpenStageDoorCommand(arm, wrist, elbow));
    }
}
