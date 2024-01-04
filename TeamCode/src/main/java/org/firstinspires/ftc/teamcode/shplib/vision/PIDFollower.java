package org.firstinspires.ftc.teamcode.shplib.vision;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PIDController;

public class PIDFollower {
    private MecanumController mecanumController;

    private PIDController yawPID;
    private PIDController pitchPID;
    private PIDController translationalPID;

    private double idealXError;
    private double idealYError;
    private double idealArea;

    public PIDFollower(PIDFollowerBuilder pidFollowerBuilder) {
        this.mecanumController = pidFollowerBuilder.mecanumController;

        this.yawPID = pidFollowerBuilder.yawPID;
        this.pitchPID = pidFollowerBuilder.pitchPID;
        this.translationalPID = pidFollowerBuilder.translationalPID;

        this.idealXError = pidFollowerBuilder.idealXError;
        this.idealYError = pidFollowerBuilder.idealYError;
        this.idealArea = pidFollowerBuilder.idealArea;
    }

    public void update(double xError, double yError, double area) {
        mecanumController.driveParams(0, 0, yawPID.getOutput(xError - idealXError));
    }

    public static class PIDFollowerBuilder {
        private MecanumController mecanumController;

        private PIDController yawPID;
        private PIDController pitchPID;
        private PIDController translationalPID;

        private double idealXError;
        private double idealYError;
        private double idealArea;

        public PIDFollowerBuilder(MecanumController mecanumController,
                                  PIDController yawPID,
                                  PIDController pitchPID,
                                  PIDController translationalPID) {

            this.mecanumController = mecanumController;

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
