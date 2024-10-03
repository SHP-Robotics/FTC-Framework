package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Drive.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.controllers.GainSchedule;
import org.firstinspires.ftc.teamcode.shplib.controllers.PositionPID;
import org.firstinspires.ftc.teamcode.shplib.hardware.drive.SHPMecanumDrive;
import org.firstinspires.ftc.teamcode.shplib.hardware.sensors.SHPIMU;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;

public class DriveSubsystem extends Subsystem {
    private final SHPMecanumDrive drive;
//    private final SHPIMU imu;

    private double bias = kMaximumBias; // will always be between kMinimumBias and 1.0

    public DriveSubsystem(HardwareMap hardwareMap) {
        drive = new SHPMecanumDrive(hardwareMap, kMotorNames);
        drive.enableBuiltInVelocityControl();

        // Change logo direction and USB direction according to your hub orientation
        // Reference pictures: https://ftc-docs.firstinspires.org/programming_resources/imu/imu.html#orthogonal-mounting
//        imu = new SHPIMU(hardwareMap,
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    }

    public void mecanum(double leftY, double leftX, double rightX) {
//        Vector2d vector = new Vector2d(
//                leftY,
//                leftX
//        ).rotated(-imu.getYaw());

//        drive.mecanum(vector.getX(), vector.getY(), rightX); // field oriented
        drive.mecanum(leftY * bias, leftX * bias, rightX * bias); // robot oriented
    }

    public void setDriveBias(double driveBias) {
        bias = Range.clip(driveBias, kMinimumBias, kMaximumBias);
    }

    public void enablePositionPID() {
        drive.enablePositionPID(0.001, 0.0, 0.0);
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
//        telemetry.addData("Bot Direction: ", Math.toDegrees(imu.getYaw()));
        for (int i = 0; i < 4; i++) {
            telemetry.addData("Motor " + i + " Position: ", drive.getPositions(MotorUnit.TICKS)[i]);
        }
//        telemetry.addData("Drive at position setpoint: ", drive.atPositionSetpoint() ? "true" : "false");
    }
}
