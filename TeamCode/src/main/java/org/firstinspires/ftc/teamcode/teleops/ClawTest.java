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
public class ClawTest extends BaseRobot {
    private double debounce;
    private double drivebias;
    // tee hee
//    private OneMotorSystem elbow;
    private Servo wrist;
//    private Servo plane;

//    private Servo climb;

    @Override
    public void init() {
        super.init();
        drivebias = 1.0;
        claw.close();
//        arm.setState(ArmSubsystem.State.INTAKE);

        // Default command runs when no other commands are scheduled for the subsystem
//        drive.setDefaultCommand(
//                new RunCommand(
//                        () -> drive.mecanum(-gamepad1.left_stick_y*drivebias, -gamepad1.left_stick_x*drivebias, -gamepad1.right_stick_x*drivebias)
//                )
//        );

//        // tee hee
//        plane = hardwareMap.get(Servo.class, "plane");

//        plane = hardwareMap.get(Servo.class, "plane");
//        plane.setPosition(0.6);

//        elbow = new OneMotorSystem.OneMotorSystemBuilder(hardwareMap, "elbow")
//                .setDirection(DcMotorSimple.Direction.REVERSE)
//                .setUseBrakes(false)
//                .setUseEncoders(true)
//                .setStaticPower(0)
//                .build();

//        elbow.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(Constants.Arm.kWristDrive);

//        climb = hardwareMap.get(Servo.class, "climb");
//        climb.setPosition(Constants.Climb.kClimbHold);

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
//        telemetry.addData("Arm state: ", arm.getState());
//        telemetry.addData("Arm pos: ", elbow.getMotorPosition());
        telemetry.addData("wrist pos: ", wrist.getPosition());
        telemetry.update();

        if (gamepad1.circle) {
            wrist.setPosition(0.1);
        }
        if (gamepad1.square) {
            wrist.setPosition(0.5);
        }
        if (gamepad1.triangle) {
            wrist.setPosition(1);
        }
    }
}
