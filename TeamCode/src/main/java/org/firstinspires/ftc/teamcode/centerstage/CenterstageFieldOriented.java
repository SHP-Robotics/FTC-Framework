package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.debug.Claw;
import org.firstinspires.ftc.teamcode.debug.LinearSlide;
import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.SpeedController;
import org.firstinspires.ftc.teamcode.debug.SpeedType;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "CenterStage Field Oriented")
public class CenterstageFieldOriented extends LinearOpMode {

    private boolean USE_WEBCAM = true;
    AprilTagLibrary myAprilTagLibrary;
    AprilTagProcessor myAprilTagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        SpeedController speedController = new SpeedController.SpeedBuilder(SpeedType.GEAR_SHIFT)
                .setGearSpacing(0.05)
                .build();

        MecanumController mecanumController = new MecanumController(hardwareMap, speedController);

        mecanumController.calibrateIMUAngleOffset();

        LinearSlide lift = new LinearSlide(hardwareMap, "lift", true);
        lift.setLiftPower(1);
        lift.applyLiftBrakes();

        Claw claw = new Claw(hardwareMap, "claw");
        claw.setRange(Constants.CLAW_OPEN, Constants.CLAW_CLOSE);

        //SampleMecanumDrive roadrunnerCorrection = new SampleMecanumDrive(hardwareMap);

        //initProcessors();

        waitForStart();

        while (opModeIsActive()) {
            mecanumController.fieldOrientedDrive(gamepad1);
            telemetry.addData("radians clockwise", mecanumController.getCalibratedIMUAngle());
            telemetry.addData("speed", speedController.getSpeed());
            telemetry.update();

            if (DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.RESET_IMU)) {
                // I wonder if this will reset the IMU if the Yaw is off after a collision
                // Further testing required
                // mecanumController.initIMU(hardwareMap);
                mecanumController.calibrateIMUAngleOffset();
            }

            lift.drive(DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.LIFT_POWER_UP)
                - DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.LIFT_POWER_DOWN));

            if (DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.CLAW_OPEN)) {
                claw.open();
            } else if (DrivingConfiguration.getValue(gamepad1, DrivingConfiguration.CLAW_CLOSE)) {
                claw.close();
            }
        }
    }
}
