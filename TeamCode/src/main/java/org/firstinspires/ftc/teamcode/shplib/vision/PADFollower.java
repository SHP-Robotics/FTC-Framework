package org.firstinspires.ftc.teamcode.shplib.vision;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PADController;
import org.firstinspires.ftc.teamcode.debug.PIDController;

public class PADFollower {
    private final MecanumController mecanumController;
    private final CRServo cameraServo;

    private final PADController yawPAD;
    private final PADController pitchPAD;
    private PADController translationalPAD;

    private final double idealXError;
    private final double idealYError;
    private double idealArea;

    public PADFollower(PADFollowerBuilder padFollowerBuilder) {
        this.mecanumController = padFollowerBuilder.mecanumController;
        this.cameraServo = padFollowerBuilder.cameraServo;

        this.yawPAD = padFollowerBuilder.yawPAD;
        this.pitchPAD = padFollowerBuilder.pitchPAD;
        this.translationalPAD = padFollowerBuilder.translationalPAD;

        this.idealXError = padFollowerBuilder.idealXError;
        this.idealYError = padFollowerBuilder.idealYError;
        this.idealArea = padFollowerBuilder.idealArea;
    }

    public void update(double xError, double yError, double area) {
        if (mecanumController != null) {
            mecanumController.driveParams(0, 0, yawPAD.getOutput(xError - idealXError));
        }

        if (cameraServo != null) {
            cameraServo.setPower(pitchPAD.getOutput(yError - idealYError));
        }
    }

    public static class PADFollowerBuilder {
        private final MecanumController mecanumController;
        private final CRServo cameraServo;

        private final PADController yawPAD;
        private final PADController pitchPAD;
        private final PADController translationalPAD;

        private double idealXError;
        private double idealYError;
        private double idealArea;

        public PADFollowerBuilder(MecanumController mecanumController,
                                  CRServo cameraServo,
                                  PADController yawPAD,
                                  PADController pitchPAD,
                                  PADController translationalPAD) {

            this.mecanumController = mecanumController;
            this.cameraServo = cameraServo;

            this.yawPAD = yawPAD;
            this.pitchPAD = pitchPAD;
            this.translationalPAD = translationalPAD;

            this.idealXError = 0;
            this.idealYError = 0;
            this.idealArea = 0;
        }

        public PADFollowerBuilder setIdealXError(double idealXError) {
            this.idealXError = idealXError;
            return this;
        }

        public PADFollowerBuilder setIdealYError(double idealYError) {
            this.idealYError = idealYError;
            return this;
        }

        public PADFollowerBuilder setIdealArea(double idealArea) {
            this.idealArea = idealArea;
            return this;
        }

        public PADFollower build() {
            return new PADFollower(this);
        }
    }
}
