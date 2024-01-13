//package org.firstinspires.ftc.teamcode.subsystems;
//
//import static org.firstinspires.ftc.teamcode.Constants.Arm.kElbowD;
//import static org.firstinspires.ftc.teamcode.Constants.Arm.kElbowG;
//import static org.firstinspires.ftc.teamcode.Constants.Arm.kElbowP;
//import static org.firstinspires.ftc.teamcode.Constants.Arm.kElbowS;
//import static org.firstinspires.ftc.teamcode.Constants.Arm.kElbowTolerance;
//import static org.firstinspires.ftc.teamcode.Constants.Lift.kLeftLiftName;
//import static org.firstinspires.ftc.teamcode.Constants.Lift.kLiftClimb;
//import static org.firstinspires.ftc.teamcode.Constants.Lift.kLiftDisable;
//import static org.firstinspires.ftc.teamcode.Constants.Lift.kLiftPrepare;
//import static org.firstinspires.ftc.teamcode.Constants.Lift.kRightLiftName;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
//import org.firstinspires.ftc.teamcode.shplib.controllers.ElevatorFFController;
//import org.firstinspires.ftc.teamcode.shplib.controllers.PositionPID;
//import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
//
//public class LiftSubsystem extends Subsystem {
//    // Declare devices
//    // Example:
//    private final SHPMotor leftLift;
//    private final SHPMotor rightLift;
//
//
//    public enum State {
//        // Define states
//        // Example:
//        DISABLED,
//        PREPARE,
//        CLIMB,
//    }
//
//    private State state;
//
//    public LiftSubsystem(HardwareMap hardwareMap) {
//        // Initialize devices
//        // Example:
//        leftLift = new SHPMotor(hardwareMap, kLeftLiftName);
//        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        leftLift.setPositionErrorTolerance(kElbowTolerance);
//
//        rightLift = new SHPMotor(hardwareMap, kRightLiftName);
//        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rightLift.setPositionErrorTolerance(kElbowTolerance);
//
//        // Set initial state
//        // Example:
//        setState(State.DISABLED);
//    }
//
//    public void setState(State state) {
//        this.state = state;
//    }
//
//    // Add control methods
//    // Example:
//    // private void setPower(double power) { motor.setPower(power); }
//
//    @Override
//    public void periodic(Telemetry telemetry) {
//        // Add logging if needed
//        // Example:
//        // telemetry.addData("Motor Encoder: ", motor.getPosition(MotorUnit.TICKS));
//
//        // Handle states
//        // Example:
//        switch (state) {
//            case PREPARE:
//                leftLift.setPosition(kLiftPrepare);
//                rightLift.setPosition(kLiftPrepare);
//                break;
//            case CLIMB:
//                leftLift.setPosition(kLiftClimb);
//                rightLift.setPosition(kLiftClimb);
//                break;
//            case DISABLED:
//                leftLift.setPosition(kLiftDisable);
//                rightLift.setPosition(kLiftDisable);
//                break;
//        }
//
//        // OR
//
////        if (state == State.ENABLED) {
////            setPower(1.0);
////        } else if (state == State.DISABLED) {
////            setPower(0.0);
////        }
//    }
//}
