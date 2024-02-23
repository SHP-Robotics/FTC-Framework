package org.firstinspires.ftc.teamcode.autos.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.VisionLocation;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class BaseAuto extends LinearOpMode {
    public enum Side {
        BLUE ("blue"),
        RED ("red");

        private final String color;

        Side(String color) {
            this.color = color;
        }
    }

    VisionSubsystem visionSubsystem;

    public Side side = Side.BLUE;
    protected VisionLocation location;

    @Override
    public void runOpMode() throws InterruptedException {
        visionSubsystem = new VisionSubsystem(hardwareMap, side.color);

        if (side == Side.BLUE) {
            location = visionSubsystem.getLocationBlue();
        } else {
            location = visionSubsystem.getLocationRed();
        }

        telemetry.addData("location", location);
        telemetry.update();

        while (opModeInInit() && !isStopRequested()) {
            if (side == Side.BLUE) {
                location = visionSubsystem.getLocationBlue();
            } else {
                location = visionSubsystem.getLocationRed();
            }
            telemetry.addData("location", location);
            telemetry.update();
        }

        waitForStart();
    }
}