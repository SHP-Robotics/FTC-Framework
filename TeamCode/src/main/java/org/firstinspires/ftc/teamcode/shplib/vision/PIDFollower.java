package org.firstinspires.ftc.teamcode.shplib.vision;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PIDController;

public class PIDFollower {
    private MecanumController mecanumController;
    private CRServo cameraServo;

    private PIDController yawPID;
    private PIDController pitchPID;
    private PIDController translationalPID;

    private double idealXError;
    private double idealYError;
    private double idealArea;

    public PIDFollower(PIDFollowerBuilder pidFollowerBuilder) {
        this.mecanumController = pidFollowerBuilder.mecanumController;
        this.cameraServo = pidFollowerBuilder.cameraServo;

        this.yawPID = pidFollowerBuilder.yawPID;
        this.pitchPID = pidFollowerBuilder.pitchPID;
        this.translationalPID = pidFollowerBuilder.translationalPID;

        this.idealXError = pidFollowerBuilder.idealXError;
        this.idealYError = pidFollowerBuilder.idealYError;
        this.idealArea = pidFollowerBuilder.idealArea;
    }

    public void update(double xError, double yError, double area) {
        if (mecanumController != null) {
            mecanumController.driveParams(0, 0, yawPID.getOutput(xError - idealXError));
        }

        if (cameraServo != null) {
            cameraServo.setPower(pitchPID.getOutput(yError - idealYError));
        }
    }

    public static class PIDFollowerBuilder {
        private MecanumController mecanumController;
        private CRServo cameraServo;

        private PIDController yawPID;
        private PIDController pitchPID;
        private PIDController translationalPID;

        private double idealXError;
        private double idealYError;
        private double idealArea;

        public PIDFollowerBuilder(MecanumController mecanumController,
                                  CRServo cameraServo,
                                  PIDController yawPID,
                                  PIDController pitchPID,
                                  PIDController translationalPID) {

            this.mecanumController = mecanumController;
            this.cameraServo = cameraServo;

            this.yawPID = yawPID;
            this.pitchPID = pitchPID;
            this.translationalPID = translationalPID;

            this.idealXError = 0;
            this.idealYError = 0;
            this.idealArea = 0;
        }

        public PIDFollowerBuilder setIdealXError(double idealXError) {
            this.idealXError = idealXError;
            return this;
        }

        public PIDFollowerBuilder setIdealYError(double idealYError) {
            this.idealYError = idealYError;
            return this;
        }

        public PIDFollowerBuilder setIdealArea(double idealArea) {
            this.idealArea = idealArea;
            return this;
        }

        public PIDFollower build() {
            return new PIDFollower(this);
        }
    }
}
