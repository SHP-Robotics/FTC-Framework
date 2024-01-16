package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous()
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        leftFront.setPower(0.3);
        sleep(1000);
        rightFront.setPower(0.3);
        sleep(1000);
        leftRear.setPower(0.3);
        sleep(1000);
        rightRear.setPower(0.3);
        sleep(1000);
    }
}
