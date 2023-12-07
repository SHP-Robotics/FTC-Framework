//package org.firstinspires.ftc.teamcode.subsystems;
//
//import static org.firstinspires.ftc.teamcode.Constants.Lift.kLiftName;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
//import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
//import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;
//
//public class LiftSubsystem extends Subsystem {
////     Declare devices
////     Example:
//     private final SHPMotor lift;
//
//    public enum State {
//        // Define states
//        // Example:
//        // ENABLED, DISABLED
//    }
//
//    private State state;
//
//    public LiftSubsystem(HardwareMap hardwareMap) {
//        // Initialize devices
//        // Example:
//         lift = new SHPMotor(hardwareMap, kLiftName);
//
//        // Set initial state
//        // Example:
//        // setState(State.TOP);
//    }
//
//    public void extend(){
//        lift.setPosition(lift.getPosition(MotorUnit.TICKS)+20);
//    }
//    public void retract(){
//        lift.setPosition(lift.getPosition(MotorUnit.TICKS)-20);
//    }
//    public void hold(){
//        lift.setPosition(lift.getPosition(MotorUnit.TICKS));
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
//         telemetry.addData("Lift : ", lift.getPosition(MotorUnit.TICKS));
//
//        // Handle states
//        // Example:
////        switch (state) {
////            case ENABLED:
////                setPower(1.0);
////                break;
////            case DISABLED:
////                setPower(0.0);
////                break;
////        }
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
