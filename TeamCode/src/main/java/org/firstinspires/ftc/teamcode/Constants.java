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

    public static final class Claw {
        public static final double kClawOpen = 0.42;
        public static final double kClawClosed = 0.52;

        public static final double kWristDown = 0.2;

        public static final double kWristDrive = 0.1;

        public static final double kWristDeposit = 0.53;

        public static final double kWristInit = 0;

    }
    public static final class Plane {
        public static final String kPlaneName = "plane";
        public static final double kPlaneLaunch = 1.0;
        public static final double kPlaneLoad = 0.6;

    }


    public static final class Lift {
        public static final String kLeftLiftName = "leftSlide";
        public static final String kRightLiftName = "rightSlide";

        public static final double kLiftDown = 0;
        public static final double kLiftDrive = 200;
        public static final double kLiftDeposit = 2200;

        public static final double kLiftP = 8;
        public static final double kLiftI = 0;
        public static final double kLiftD = 4;

    }
}
