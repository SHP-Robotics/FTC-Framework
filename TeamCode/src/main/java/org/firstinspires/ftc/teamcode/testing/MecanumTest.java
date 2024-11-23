package org.firstinspires.ftc.teamcode.testing;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class MecanumTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor fl = (DcMotor) hardwareMap.get("frontLeft");
        DcMotor fr = (DcMotor) hardwareMap.get("frontRight");
        DcMotor bl = (DcMotor) hardwareMap.get("backLeft");
        DcMotor br = (DcMotor) hardwareMap.get("backRight");

        fl.setDirection(FORWARD);
        fr.setDirection(REVERSE);
        bl.setDirection(FORWARD);
        br.setDirection(REVERSE);

        waitForStart();

        fl.setPower(0.3);
        sleep(500);
        fr.setPower(0.3);
        sleep(500);
        bl.setPower(0.3);
        sleep(500);
        br.setPower(0.3);
        sleep(500);
    }
}
