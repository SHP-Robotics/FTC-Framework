package org.firstinspires.ftc.teamcode.shplib;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.shplib.controllers.FFController;

@Config
public class Constants {
    // Target voltage for voltage compensation
    public static final double kNominalVoltage = 12.0;
    public static double offset = 0;
    public static final class Drive {
        public static final String[] kMotorNames = new String[]{
                "leftFront",
                "leftRear",
                "rightFront",
                "rightRear"
        };
        public static final FFController[] kFFs = new FFController[]{
                new FFController(0.07),
                new FFController(0.07), //TODO: ??
                new FFController(0.07),
                new FFController(0.045)
        };
        public static final double kMinimumBias = 0.3;
        public static final double kMaximumBias = 0.6;
    }

    public static final class Vision {
        public static final double kTagsizeMeters = 0.0475;
    }

    public static final class Intake{
//        public static final String kLeftAdjust = "LeftAxon";
//        public static final String kRightAdjust = "RightAxon";
//        public static final double kPositionBottom = 0.615;

        // Wrist
        public static double kWristDown = 0.405;
        public static double kWristUp = 0.3;
        public static double kWristHalfway = 1;
        public static double kWristClimb = 0.4;
        public static double kWristStageDoor = 0.4;

        // Elbow
        public static double kPositionBottom = 0.56;
        public static double kPositionTop = 0.05; //0.05
        public static double kPositionMiddle = 0.3;
        public static double kPositionStageDoor = 0.335;

        public static final String kPixelServo = "MiniServo";
        public static final String kPlaneServo = "plane";

        public static final String kAdjustHolder = "35kg";

        public static final String kSpinningIntakeName = "SpinningIntake";
        public static final String kCRWheelName = "CRWheel";
        public static final double kWheelForward = 1.0;
        public static final double kWheelBackward = -1.0;
        public static final double kWheelStill = 0.0;

        // Hook Servo Constants
        public static final String kHookServo1Name = "Hook1";
        public static final String kHookServo2Name = "Hook2";
        public static final double kHookLeftEngaged = 0.85;
        public static final double kHookLeftDisengaged = 0.5;
        public static final double kHookRightEngaged = 0.0;
        public static final double kHookRightDisengaged = 0.5;

        public static final String kPracticeLeftArmServoName = "LeftAxon";
        public static final String kPracticeRightArmServoName = "RightAxon";
    }

    public static final class Arm {
        public static final String kLeftSlideName = "leftSlide";
        public static final String kRightSlideName = "rightSlide";

        public static final double kMaxHeight = 4000;
        public static final double kSlideTolerance = 20;
        public static final double kPixelHeight = 500;
        public static final double kRunPower = 1;
    }
}
