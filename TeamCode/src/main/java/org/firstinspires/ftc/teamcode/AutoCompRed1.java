package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.shprobotics.pestocore.drivebases.MecanumController;
//import com.shprobotics.pestocore.drivebases.MecanumTracker;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;

//@Autonomous(name = "AutoCompRed1", group = "Autonomous") public class AutoCompRed1 extends LinearOpMode {

@Autonomous
public class AutoCompRed1 extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor wormGearMotor;
    private DcMotor strongArmMotor;
    private DcMotor strongArm2Motor;
    private DcMotor viperslide;
    private Servo claw;
    private Servo spinner;

    @Override
    public void runOpMode() {
        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        ThreeWheelOdometryTracker threeWheelOdometryTracker = PestoFTCConfig.getTracker(hardwareMap);


        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        strongArmMotor = hardwareMap.get(DcMotor.class, "strongArm");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        //ALL SPIN FORWARD (Technically)

        waitForStart();

        while (opModeIsActive())  {

            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);
            //Drive Forward

            sleep(1500);

            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
            //Turn Left

            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);
            //Drive Forward

            sleep(1500);

            frontLeft.setPower(0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(0.5);
            backRight.setPower(-0.5);
            //Turn Right (Technically)

            strongArmMotor.setTargetPosition(-1104);
            strongArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        }
    }
}