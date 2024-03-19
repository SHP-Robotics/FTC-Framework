package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@Disabled
@TeleOp(name = "Binary Search")
public class BinarySearch extends LinearOpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;

    public void setPower(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        boolean holdingA = false;
        boolean holdingY = false;

        double l = 0;
        double r = 1;
        double m = (l + r) / 2;



        waitForStart();

        setPower(m);
        telemetry.addLine("Wheels at " + String.valueOf(m) + "volts power");
        telemetry.update();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                if (!holdingY) {
                    holdingY = true;
                    l = m;
                    m = (l + r) / 2;
                    setPower(m);
                    telemetry.addLine("Wheels at " + String.valueOf(m) + "volts power");
                    telemetry.update();
                }
            } else {
                holdingY = false;
            }

            if (gamepad1.a) {
                if (!holdingA) {
                    holdingA = true;
                    r = m;
                    m = (l + r) / 2;
                    setPower(m);
                    telemetry.addLine("Wheels at " + String.valueOf(m) + "volts power");
                    telemetry.update();
                }
            } else {
                holdingA = false;
            }
        }
    }
}
