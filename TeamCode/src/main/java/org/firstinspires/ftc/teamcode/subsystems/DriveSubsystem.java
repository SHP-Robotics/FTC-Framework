package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class DriveSubsystem extends Subsystem {
    public final MecanumController drive;
    public final DeterministicTracker tracker;
    public final TeleOpController teleOpController;

    public DriveSubsystem(HardwareMap hardwareMap) {
        drive = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        teleOpController = PestoFTCConfig.getTeleOpController(drive, tracker, hardwareMap);
    }

    public void update(Gamepad gamepad) {
        this.teleOpController.updateSpeed(gamepad);
        this.tracker.update();
    }

    public void mecanum(double forward, double strafe, double rotate) {
        teleOpController.driveFieldCentric(forward, strafe, rotate);
    }
    public void resetIMUAngle() {
//        teleOpController.resetIMU();
        tracker.reset();
    }

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("IMU ANGLE:", teleOpController.getHeading());
    }
}
