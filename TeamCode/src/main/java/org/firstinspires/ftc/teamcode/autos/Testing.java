package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

@Autonomous
public class Testing extends BaseRobot {
    private double debounce;
    @Override
    public void init(){
        super.init();
//        driveBias = 1;
//
//        drive.setDefaultCommand(
//                new RunCommand(
//                        () -> drive.mecanum(-gamepad1.left_stick_y*driveBias, -gamepad1.left_stick_x*driveBias, gamepad1.right_stick_x*driveBias)
//                )
//        );
//        wrist.setState(WristSubsystem.State.MANUAL);
        //justPressed = false;
//        slideMotor = (DcMotorEx) hardwareMap.get("slide");
//        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slide.setDefaultCommand(
//                new RunCommand(
//                        () -> {if(!gamepad1.left_bumper && !gamepad1.right_bumper)
//                            slide.setPower(0);
//
//                        }
//                )
//        );
//        elbowPos=0;

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
//        telemetry.addData("Slide Direction:", slideMotor.getDirection());
//        telemetry.addData("Slide Mode:", slideMotor.getMode());
//        telemetry.addData("Slide Pos: ", (float)slideMotor.getCurrentPosition());

        telemetry.update();

//        new Trigger(gamepad1.right_trigger > 0.5, new RunCommand(() -> {
//            intake.getServo().setPower(1.0);
//        }));
//        new Trigger(gamepad1.left_trigger > 0.5, new RunCommand(() -> {
//            intake.getServo().setPower(-1.0);
//        }));

//        new Trigger(gamepad1.dpad_up, new RunCommand(
//                ()-> {wrist.incrementUp();}
//        ));
//        new Trigger(gamepad1.dpad_down, new RunCommand(
//                ()-> {wrist.incrementDown();}
//        ));
//        new Trigger(gamepad1.triangle, new RunCommand(
//                ()-> {slide.incrementUp();}
//        ));
//        new Trigger(gamepad1.x, new RunCommand(
//                ()-> {slide.incrementDown();}
//        ));

//        new Trigger(gamepad1.right_bumper, new RunCommand(
//                () -> {
//
//                    slide.setPower(0.5);
//
//
//                }));
//        new Trigger(gamepad1.left_bumper, new RunCommand(
//                () -> {
//
//
//                    slide.setPower(-0.5);
//
//
//                }));
//
//        new Trigger(gamepad1.triangle,
//                new RunCommand(
//                        ()-> {intake.getServo().setPower(1.0);}
//                )
//        );
//
//        elbow.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
//
        intake.getServo().setPower(1.0);
        debounce = Clock.now();
    }
}
