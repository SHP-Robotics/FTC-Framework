package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.shplib.controllers.FFController;

@Config
public class Constants {
    // Target voltage for voltage compensation
    public static final double kNominalVoltage = 12.0;

    public static final class Drive {
        public static final String[] kMotorNames = new String[]{
                "leftFront",
                "leftRear",
                "rightFront",
                "rightRear"
        };
        public static final FFController[] kFFs = new FFController[]{
                new FFController(0.047),
                new FFController(0.047),
                new FFController(0.045),
                new FFController(0.065)
        };
        public static final double kMinimumBias = 0.3;
        public static final double kMaximumBias = 1.0;
    }

    public static final class Vision {
        public static final double kTagsizeMeters = 0.0475;
    }

    public static final class Arm {

        public static final String kElbowName = "elbow";
        public static final String kWristName = "wrist";
        public static final double kElbowDown = 0;
        public static final double kElbowDrive = 100;

        public static final double kElbowUp = 1300;

        public static final double kWristDrive = 0.8;
        public static final double kWristDown = 0.5;
        public static final double kWristDeposit = 1;



        public static final double kElbowP = 0.0018;
        public static final double kElbowD = 0;
        public static final double kElbowTolerance = 20;

        public static final double kElbowS = 0.035; // static friction
        public static final double kElbowG = 0.07; // gravity

    }

    public static final class Claw {
        public static final String kLeftClawName = "leftClaw";
        public static final double kLeftClawOpen = .2; //FINAL
        public static final double kLeftClawClosed = .45; //bigger in
        public static final String kRightClawName = "rightClaw";
        public static final double kRightClawOpen = 0.25; //FINAL
        public static final double kRightClawClosed = 0.0; //smaller in

    }
    public static final class Plane {
        public static final String kPlaneName = "plane";
        public static final double kPlaneLaunch = 1.0;
        public static final double kPlaneLoad = 0.6;

    }

    public static final class Climb {
        public static final double kClimbHold = 0.5;

        public static final double kClimbRelease = 0.2;
    }
//    public static final class Lift {
//        public static final String kLeftLiftName = "leftLift";
//        public static final String kRightLiftName = "rightLift";
//
//        public static final double kLiftDisable = 0.0475;
//        public static final double kLiftPrepare = 0.0475;
//        public static final double kLiftClimb = 0.0475;
//
//    }
}
