package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.AccumulationControlledServo;

@TeleOp()
public class CRServoTankDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo left = (CRServo) hardwareMap.get("left");
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        CRServo right = (CRServo) hardwareMap.get("right");
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        AccumulationControlledServo leftPlow = new AccumulationControlledServo.AccumulationControlledServoBuilder((Servo) hardwareMap.get("leftPlow"))
                .setkP(1)
                .build();

        AccumulationControlledServo rightPlow = new AccumulationControlledServo.AccumulationControlledServoBuilder((Servo) hardwareMap.get("rightPlow"))
                .setkP(1)
                .build();

        Servo eye = (Servo) hardwareMap.get("eye");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            left.setPower(-gamepad1.left_stick_y);
            right.setPower(-gamepad1.right_stick_y);

            if (gamepad1.square){
                eye.setPosition(0.5);
            }
            if (gamepad1.triangle){
                eye.setPosition(0.345);
            }

            if (gamepad1.circle){
                eye.setPosition(0.25);
            }
            
            leftPlow.setPosition(gamepad1.left_trigger);
            rightPlow.setPosition(-gamepad1.left_trigger);
        }
    }
}
