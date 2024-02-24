package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@TeleOp
public class PosTuner extends BaseRobot {
    // spammable code
    private boolean holdingUp = false;
    private boolean holdingRight = false;
    private boolean holdingDown = false;

    private boolean changingWrist = true;

//    Servo servo;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        super.start();

//        servo = (Servo) hardwareMap.get("dropdown");

        elbow.setState(ElbowSubsystem.State.DOWN);
        wrist.setState(WristSubsystem.State.DOWN);
    }

    @Override
    public void loop() {
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

//        servo.setPosition(gamepad1.left_stick_y);
//        telemetry.addData("pos", servo.getPosition());
        telemetry.update();
    }
}
