package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Drive.*;
import static org.firstinspires.ftc.teamcode.commands.EncoderStraightDriveCommand.encoderTicksToInches;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.controllers.GainSchedule;
import org.firstinspires.ftc.teamcode.shplib.controllers.PositionPID;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.drive.SHPMecanumDrive;
import org.firstinspires.ftc.teamcode.shplib.hardware.sensors.SHPIMU;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;

public class DriveSubsystem extends Subsystem {
    private final SHPMecanumDrive drive;
    public final Encoder parallelEncoder, perpendicularEncoder;
    public final SHPIMU imu;

    private double bias = 1; // will always be between kMinimumBias and 1.0

    public DriveSubsystem(HardwareMap hardwareMap) {
        drive = new SHPMecanumDrive(hardwareMap, kMotorNames);
        drive.enableBuiltInVelocityControl();
        for (int i = 0; i<4; i++) {
            //drive.motors[i].enablePositionPID(K_DRIVE_P1);
            drive.motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Change logo direction and USB direction according to your hub orientation
        // Reference pictures: https://ftc-docs.firstinspires.org/programming_resources/imu/imu.html#orthogonal-mounting
        imu = new SHPIMU(hardwareMap,
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelEncoder"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicularEncoder"));

    }

    public void mecanum(double leftY, double leftX, double rightX) {
        Vector2d vector = new Vector2d(
                leftY,
                leftX
        ).rotated(-(imu.getYaw(AngleUnit.RADIANS)));

        drive.mecanum(vector.getX(),vector.getY(), rightX); // field oriented
        //drive.mecanum(leftY * bias, leftX * bias, rightX * bias); // robot oriented
    }
    public void robotmecanum(double leftY, double leftX, double rightX) {
        //drive.mecanum(vector.getX(),vector.getY(), rightX); // field oriented
        drive.mecanum(leftY * bias, leftX * bias, rightX * bias); // robot oriented
    }

    public void setDriveBias(double driveBias) {
        bias = driveBias;
    }

    public void enablePositionPID() {
        //drive.enablePositionPID(0.001, 0.0, 0.0);
        drive.enablePositionPID(5, 0.0, 0.0);
        drive.scheduleGains(
                new GainSchedule(0.0005, 0.0, 0.0, 0.80),
                new GainSchedule(0.0002, 0.0, 0.0, 0.40)
//                new GainSchedule(0.0001, 0.0, 0.0, 0.10)

//                new GainSchedule(0.00015, 0.0, 0.0, 0.20)
        );
        drive.enableFF(kFFs);
    }

    public void setInitialPositions() {
        drive.setInitialPositions(MotorUnit.TICKS);
    }
//
//    public void driveTo(boolean usingPID, double ticks) {
//        if (usingPID) {
//            drive.setPosition(ticks);
//        } else {
//            while (drive.getWheelPositionsAveraged(MotorUnit.TICKS) <= ticks) {
//                drive.mecanum(0.5, 0.5, 0);
//            }
//            drive.mecanum(0.0, 0.0, 0);
//        }
//    }

//    public boolean atPositionSetpoint() {
//        return drive.atPositionSetpoint();
//    }

    public void setPosition(double position) {
        drive.setPositions(position);
    }

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("Y Position: ", encoderTicksToInches(parallelEncoder.getCurrentPosition()));
        telemetry.addData("X Position: ", encoderTicksToInches(perpendicularEncoder.getCurrentPosition()));
        telemetry.addData("Robot Degrees: ", imu.getYaw(AngleUnit.DEGREES));
        for (int i = 0; i < 4; i++) {
            //telemetry.addData("Motor " + i + " Position: ", drive.getPositions(MotorUnit.TICKS)[i]);
        }

//        telemetry.addData("Drive at position setpoint: ", drive.atPositionSetpoint() ? "true" : "false");
    }
}
