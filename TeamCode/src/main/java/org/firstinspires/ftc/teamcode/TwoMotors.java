package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class TwoMotors extends LinearOpMode {
    public static double motorPower = 0.5;

    public void runOpMode() {
        DcMotor motor1 = (DcMotor)hardwareMap.get("left");
        DcMotor motor2 = (DcMotor)hardwareMap.get("right");

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            motor1.setPower(motorPower);
            motor2.setPower(motorPower);
        }

        motor1.setPower(0);
        motor2.setPower(0);
    }
}
