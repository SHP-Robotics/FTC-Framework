package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

@TeleOp
public class PosTuner extends BaseRobot {
    private double debounce;
    private double driveBias;

    // spammable code
    private boolean holdingUp = false;
    private boolean holdingRight = false;
    private boolean holdingDown = false;

    private boolean changingWrist = true;

//    Servo servo;

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

//        servo = (Servo) hardwareMap.get("dropdown");
    }

    @Override
    public void loop() {
        // Allows CommandScheduler.run() to be called - DO NOT DELETE!
        super.loop();

        if (gamepad1.dpad_up) {
            if (!holdingUp) {
                if (changingWrist) {
                    Constants.Intake.kWristDown += 0.005;
                } else {
                    Constants.Intake.kPositionBottom += 0.005;
                }
            }
            holdingUp = true;
        } else {
            holdingUp = false;
        }

        if (gamepad1.dpad_down) {
            if (!holdingDown) {
                if (changingWrist) {
                    Constants.Intake.kWristDown -= 0.005;
                } else {
                    Constants.Intake.kPositionBottom -= 0.005;
                }
            }
            holdingDown = true;
        } else {
            holdingDown = false;
        }

        if (gamepad1.dpad_right) {
            if (!holdingRight) {
                changingWrist = !changingWrist;
            }
            holdingRight = true;
        } else {
            holdingRight = false;
        }

        telemetry.addData("kWristDown", Constants.Intake.kWristDown);
        telemetry.addData("kPositionBottom", Constants.Intake.kPositionBottom);
//
//        servo.setPosition(gamepad1.left_stick_y);
//        telemetry.addData("pos", servo.getPosition());
//        telemetry.update();
    }
}
