package org.firstinspires.ftc.teamcode.shplib.vision;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.AccumulationController;
import org.firstinspires.ftc.teamcode.debug.PIDController;

public class PIADFollower {
    private MecanumController mecanumController;
    private Servo cameraServo;

    private PIDController yawPID;
    private AccumulationController pitchPAD;
    private PIDController translationalPID;

    private double idealXError;
    private double idealYError;
    private double idealArea;

    public PIADFollower(PIADFollowerBuilder piadFollowerBuilder) {
        this.mecanumController = piadFollowerBuilder.mecanumController;
        this.cameraServo = piadFollowerBuilder.cameraServo;

        this.yawPID = piadFollowerBuilder.yawPID;
        this.pitchPAD = piadFollowerBuilder.pitchPAD;
        this.translationalPID = piadFollowerBuilder.translationalPID;

        this.idealXError = piadFollowerBuilder.idealXError;
        this.idealYError = piadFollowerBuilder.idealYError;
        this.idealArea = piadFollowerBuilder.idealArea;
    }

    private static double clamp(double x, double low, double high) {
        return Math.max(low, Math.min(high, x));
    }

    public void update(double xError, double yError, double area) {
        if (mecanumController != null) {
            mecanumController.driveParams(0, 0, yawPID.getOutput(xError - idealXError));
        }

        if (cameraServo != null) {
            cameraServo.setPosition(pitchPAD.getOutput(yError - idealYError));
        }
    }

    public static class PIADFollowerBuilder {
        private MecanumController mecanumController;
        private Servo cameraServo;

        private PIDController yawPID;
        private AccumulationController pitchPAD;
        private PIDController translationalPID;

        private double idealXError;
        private double idealYError;
        private double idealArea;

        public PIADFollowerBuilder(MecanumController mecanumController,
                                  Servo cameraServo,
                                  PIDController yawPID,
                                  AccumulationController pitchPAD,
                                  PIDController translationalPID) {

            this.mecanumController = mecanumController;
            this.cameraServo = cameraServo;

            this.yawPID = yawPID;
            this.pitchPAD = pitchPAD;
            this.translationalPID = translationalPID;

            this.idealXError = 0;
            this.idealYError = 0;
            this.idealArea = 0;
        }

        public PIADFollowerBuilder setIdealXError(double idealXError) {
            this.idealXError = idealXError;
            return this;
        }

        public PIADFollowerBuilder setIdealYError(double idealYError) {
            this.idealYError = idealYError;
            return this;
        }

        public PIADFollowerBuilder setIdealArea(double idealArea) {
            this.idealArea = idealArea;
            return this;
        }

        public PIADFollower build() {
            return new PIADFollower(this);
        }
    }
}
