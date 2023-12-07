package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.shplib.controllers.FFController;

@Config
public class Constants {
    // Target voltage for voltage compensation
    public static final double kNominalVoltage = 12.0;


    //DEADWHEELS:
    //Rightside: Control Pos 3.
    //Front: Expansion pos 1
    //leftside: expansion pos 2
    public static final class Drive {
        public static final String[] kMotorNames = new String[]{
                "leftFront",
                "leftRear",
                "rightFront", //control pos 2
                "rightRear" //control pos 1
        };
//        public static final FFController[] kFFs = new FFController[]{ //feedforward
//                new FFController(0.047),
//                new FFController(0.07),
//                new FFController(0.045),
//                new FFController(0.035)
//        };
        public static final double kMinimumBias = 0.0;
        public static final double kMaximumBias = 0.15;
    }

    public static final class Vision {
        public static final double kTagsizeMeters = 0.0475;
    }

    public static final class Plane {
        public static final String kPlaneName = "planeServo"; //pos 0

        public static final String kHexagonName = "hexagonServo"; //pos 1
        public static final String kMissileLauncherName = "missileServo"; //pos 2

    }
//    public static final class Lift {
//        public static final String kLiftName = "lift"; //ExHub pos 0
//
//    }

    public static final class Intake {
        public static final String kIntakeName = "intake"; //pos 1

    }

//    public static final class Arm {
//        public static final String kClawName = "claw";
//        public static final double kClawOpen = 0.4;
//        public static final double kClawClosed = 0.7;
//
//        public static final String kLeftSlideName = "leftSlide";
//        public static final String kRightSlideName = "rightSlide";
//
//        public static final double kSlideBottom = 10.0;
//        public static final double kSlideHub = 200.0;
//        public static final double kSlideLow = 1000.0;
//        public static final double kSlideMiddle = 2500.0;
//        public static final double kSlideHigh = 4000.0;
//        public static final double kSlideStackDistance = 150.0;
//
//        public static final double kSlideP = 0.0018;
//        public static final double kSlideD = 0;
//        public static final double kSlideTolerance = 100;
//
//        public static final double kSlideS = 0.035; // static friction
//        public static final double kSlideG = 0.07; // gravity
//    }
}
