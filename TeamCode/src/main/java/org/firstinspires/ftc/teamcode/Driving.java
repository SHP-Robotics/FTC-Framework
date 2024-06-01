package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;

@TeleOp
public class Driving extends LinearOpMode {
    public void runOpMode() {
        MecanumController mecanumController = new MecanumController(hardwareMap, new String[]{
                "frontLeft",
                "frontRight",
                "backLeft",
                "backRight"
        });

        mecanumController.configureMotorDirections(new DcMotorSimple.Direction[]{
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD
        });

        TeleOpController teleOpController = new TeleOpController(mecanumController, hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }
}
