package org.firstinspires.ftc.teamcode.shplib;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.shplib.controllers.FFController;

@Config
public class Constants {
    // Target voltage for voltage compensation
    public static final double kNominalVoltage = 13.0;

    public static final class Drive {
        public static final String[] kMotorNames = new String[]{
                "leftFront",
                "leftRear",
                "rightFront",
                "rightRear"
        };

        public static final FFController[] kFFs = new FFController[]{
                new FFController(0.07),
                new FFController(0.07),
                new FFController(0.07),
                new FFController(0.07)
        };

        public static final double kMinimumBias = 0;
        public static final double kMaximumBias = 0.6;
    }

    public static final class Sensors{
        public static final String kClawColorName = "clawColor";
    }
//    public static final class Vision {
//        public static final double kTagsizeMeters = 0.0475;
//    }

    public static final class Pivot{

        public static final String kWristName = "wrist";
        public static final String klElbowName = "lElbow";
        public static final String krElbowName = "rElbow";
    }

    public static final class Vertical {
        public static final String kLeftSlideName = "leftVSlide";
        public static final String kRightSlideName = "rightVSlide";

        public static final double kMaxHeight = 2300;
        public static final double kSlideTolerance = 5;
        public static final double kIncrement = 25;
        public static final double kRunPower = 1;
    }

    public static final class Horiz {
        public static final String kLeftHorizSlideName = "lHoriz";
        public static final String kRightHorizSlideName = "rHoriz";
        public static final String kRailName = "rail";
    }
    public static final class Rotate{
        public static final String kRotateName = "rotateServo";
        public static final double kPickup = 0.35;
        public static final double kNeutral = 0.9;
    }

    public static final class Claw{
        public static final String kClawName = "clawServo";
        public static final double kOpen = 0.45;
        public static final double kClose = 0;
    }
}
