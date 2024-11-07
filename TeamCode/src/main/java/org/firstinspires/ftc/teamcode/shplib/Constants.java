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

        public static final double kMinimumBias = 0.4;
        public static final double kMaximumBias = 1;
    }

    public static final class Vision {
        public static final double kTagsizeMeters = 0.0475;
    }


    public static final class Pivot{
        public static final class Intake{
            public static final String kIntakeName = "intake";
        }
        public static final String kWristName = "wrist";
        public static final String klElbowName = "lElbow";
        public static final String krElbowName = "rElbow";
        public static double kWristPos = 0.5;
        public static double kElbowIn = 0;
        public static double kElbowOut = 0.7;
    }
   // public static final class Intake{
       // public static final String kIntakeName = "intakeServo";
//        public static final String kLeftAdjust = "LeftAxon";
//        public static final String kRightAdjust = "RightAxon";
//        public static final double kPositionBottom = 0.615;


        // Wrist
//        public static double kWristDown = 0.505;
//        public static double kWristUp = 0.3;
//        public static double kWristHalfway = 0.9;
//        public static double kWristClimb = 0.4;
//        public static double kWristStageDoor = 0.4;
//        public static double kWristMosaic = 0.765;

        // Elbow
//        public static double kPositionBottom = 0.565;
//        public static double kPositionTop = 0.05; //0.05
//        public static double kPositionMiddle = 0.3;
//        public static double kPositionStageDoor = 0.335;
//        public static double kPositionMosaic = 0.045;
//
//        public static final String kPixelServo = "MiniServo";
//
//        public static final String kAdjustHolder = "35kg";
//
//        public static final String kSpinningIntakeName = "SpinningIntake";
//        public static final String kCRWheelName = "CRWheel";
//
//        public static final String kPracticeLeftArmServoName = "LeftAxon";
//        public static final String kPracticeRightArmServoName = "RightAxon";
//    }

    public static final class Arm {
        public static final String kLeftSlideName = "leftSlide";
        public static final String kRightSlideName = "rightSlide";

        public static final double kMaxHeight = 4000;
        public static final double kSlideTolerance = 30;
        public static final double kPixelHeight = 500;
        public static final double kRunPower = 1;
    }

    public static final class DropDown {
        public static final String kDropdownName = "dropdown";
        // the positions are in subsystems/DropDownSubsystem/State
    }
    public static final class Claw{
        public static final String kClawName = "clawServo";
        public static final double kOpen = -0.5;
        public static final double kClose = 0.5;
    }
}
