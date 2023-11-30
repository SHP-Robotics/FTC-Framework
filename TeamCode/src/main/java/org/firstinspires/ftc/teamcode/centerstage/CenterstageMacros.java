package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.OneMotorSystem;
import org.firstinspires.ftc.teamcode.debug.config.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class CenterstageMacros {
    MecanumController mecanumController;
    OneMotorSystem lift;
    Servo claw;

    AprilTagProcessor aprilTagProcessor;
    ColorSensor colorSensor;

    public CenterstageMacros(CenterstageRobotBuilder centerstageRobotBuilder) {
        this.mecanumController = centerstageRobotBuilder.mecanumController;
        this.aprilTagProcessor = centerstageRobotBuilder.aprilTagProcessor;
        this.colorSensor = centerstageRobotBuilder.colorSensor;
    }

    public static class CenterstageRobotBuilder {
        MecanumController mecanumController;
        OneMotorSystem lift;
        Servo claw;

        AprilTagProcessor aprilTagProcessor;
        ColorSensor colorSensor;

        public CenterstageRobotBuilder(MecanumController mecanumController) {
            this.mecanumController = mecanumController;
        }

        public CenterstageRobotBuilder setLift(OneMotorSystem lift) {
            this.lift = lift;
            return this;
        }

        public CenterstageRobotBuilder setClaw(Servo claw) {
            this.claw = claw;
            return this;
        }

        public CenterstageRobotBuilder setAprilTagProcessor(AprilTagProcessor aprilTagProcessor) {
            this.aprilTagProcessor = aprilTagProcessor;
            return this;
        }

        public CenterstageRobotBuilder setColorSensor(ColorSensor colorSensor) {
            this.colorSensor = colorSensor;
            return this;
        }

        public CenterstageMacros build() {
            return new CenterstageMacros(this);
        }
    }

    public void alignWithAprilTag(int id) {
        int count = 0;
        double avgX = 0;
        double avgY = 0;

        ElapsedTime oneSecondTimer = new ElapsedTime();
        oneSecondTimer.reset();

        while (oneSecondTimer.seconds() < 1) {
            AprilTagPoseFtc position = getAprilTagPosition(id);
            if (position != null) {
                // TODO: find out the unit type of position (inches, mm, unknown?)
                // TODO: find out how vector (x, y, z) corresponds with vector (right, up, forward)
                avgX += position.x;
                avgY += position.z;
                count += 1;
            }
        }

        if (count != 0) {
            avgX /= count;
            avgY /= count;
        }

        this.mecanumController.fakeReset();
        this.mecanumController.moveToPosition(avgX, avgY, true);
    }

    public AprilTagPoseFtc getAprilTagPosition(int... ids) {
        if (this.aprilTagProcessor == null) {
            throw new IllegalArgumentException("CenterstageRobotBuilder must be passed an instance of an AprilTagProcessor if using april tag positions.");
        }

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                for (int id: ids) {
                    if (detection.id == id) {
                        return detection.ftcPose;
                    }
                }
            }
        }

        return null;
    }

    public void dropPurplePixel() {
        if (this.lift == null || this.claw == null) {
            throw new IllegalArgumentException("CenterstageRobotBuilder must be passed an instance of a lift and claw if using pixel dropping macros.");
        }

        claw.setPosition(Constants.CLAW_OPEN);
        lift.setPosition((int) (Constants.LIFT_ENCODER_TICKS_PER_INCH * Constants.SHALLOW_PIXEL_CLAW_HEIGHT), true);
        lift.setLiftPower(0.3);

        claw.setPosition(org.firstinspires.ftc.teamcode.debug.config.Constants.CLAW_CLOSE);
        lift.setPosition((int) (Constants.LOW_BONUS_HEIGHT), true);
        lift.setLiftPower(0.3);

        claw.setPosition(org.firstinspires.ftc.teamcode.debug.config.Constants.CLAW_OPEN);

        lift.setPosition((int) (Constants.LIFT_ENCODER_TICKS_PER_INCH * Constants.DEEP_PIXEL_CLAW_HEIGHT), true);
        lift.setLiftPower(0);
    }

    public void dropYellowPixel() {
        if (this.lift == null || this.claw == null) {
            throw new IllegalArgumentException("CenterstageRobotBuilder must be passed an instance of a lift and claw if using pixel dropping macros.");
        }

        claw.setPosition(org.firstinspires.ftc.teamcode.debug.config.Constants.CLAW_OPEN);
        lift.setPosition((int) (Constants.LIFT_ENCODER_TICKS_PER_INCH * Constants.DEEP_PIXEL_CLAW_HEIGHT), true);
        lift.setLiftPower(0.3);

        claw.setPosition(org.firstinspires.ftc.teamcode.debug.config.Constants.CLAW_CLOSE);
        lift.setPosition((int) (Constants.LOW_BONUS_HEIGHT), true);
        lift.setLiftPower(0.3);

        claw.setPosition(org.firstinspires.ftc.teamcode.debug.config.Constants.CLAW_OPEN);

        lift.setPosition((int) (Constants.LIFT_ENCODER_TICKS_PER_INCH * Constants.DEEP_PIXEL_CLAW_HEIGHT), true);
        lift.setLiftPower(0);
    }
}
