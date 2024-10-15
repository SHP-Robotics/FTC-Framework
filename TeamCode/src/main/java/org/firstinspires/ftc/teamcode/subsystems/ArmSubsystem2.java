package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Arm.kClawName;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kSlideD;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kSlideP;
import static org.firstinspires.ftc.teamcode.Constants.Arm2.kClaw2Name;
import static org.firstinspires.ftc.teamcode.Constants.Arm2.kClawOpen;
import static org.firstinspires.ftc.teamcode.Constants.Arm2.kElbowDrive;
import static org.firstinspires.ftc.teamcode.Constants.Arm2.kElbowIntake;
import static org.firstinspires.ftc.teamcode.Constants.Arm2.kElbowOuttake;
import static org.firstinspires.ftc.teamcode.Constants.Arm2.kExtensionDrive;
import static org.firstinspires.ftc.teamcode.Constants.Arm2.kExtensionIntake;
import static org.firstinspires.ftc.teamcode.Constants.Arm2.kExtensionOuttake;
import static org.firstinspires.ftc.teamcode.Constants.Arm2.kWristName;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.controllers.PositionPID;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;

public class ArmSubsystem2 extends Subsystem {
    // Declare devices
    // Example:
    // private final SHPMotor motor;
    private final SHPMotor elbow;
    private final Servo wrist;
    private final Servo claw;

    private final SHPMotor extension;

    public enum State {
        // Define states
        // Example:
        INTAKE, DRIVING, OUTTAKE

    }

    private State state;

    public ArmSubsystem2(HardwareMap hardwareMap) {
        // Initialize devices
        // Example:
        // motor = new SHPMotor(hardwareMap, "motor");
        elbow = new SHPMotor(hardwareMap, "elbow");
        extension = new SHPMotor(hardwareMap, "extension");
        wrist= hardwareMap.get(Servo.class,kWristName);
        claw= hardwareMap.get(Servo.class,kClaw2Name);

        wrist.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);
//        elbow.enablePositionPID(new PositionPID(kSlideP, 0.0, kSlideD));
//        extension.enablePositionPID(new PositionPID(kSlideP, 0.0, kSlideD));

        // Set initial state
        // Example:
//         setState(State.TOP);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        elbow.setDirection(DcMotorSimple.Direction.FORWARD);
        extension.setDirection(DcMotorSimple.Direction.FORWARD);

        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        state=State.DRIVING;

        setState(State.DRIVING);

    }
    public void setState(State state){
        this.state=state;
    }
    public State getState(){
        return state;
    }
    public void setElbowPos(int pos) {
        elbow.setPosition(pos);
    }
    public void setExtensionPos(int pos) {
        extension.setPosition(pos);
    }


    public void openclaw() {
        claw.setPosition(kClawOpen);

    }





    // Add control methods
    // Example:
    // private void setPower(double power) { motor.setPower(power); }
    private void processState(){
        if (this.state==State.DRIVING){
            extension.setPosition(kExtensionDrive);
            elbow.setPosition(kElbowDrive);
        }
        if (this.state==State.INTAKE){
            extension.setPosition(kExtensionIntake);
            elbow.setPosition(kElbowIntake);
        }
        if (this.state==State.OUTTAKE){
            extension.setPosition(kExtensionOuttake);
            elbow.setPosition(kElbowOuttake);
        }

    }
    public void incrementUp() {
        if (state==State.INTAKE) {
            state=State.DRIVING;

        }
        if (state==State.DRIVING) {
            state=State.OUTTAKE;

        }
    }

    public void incrementDown() {
        if (state==State.DRIVING) {
            state=State.INTAKE;

        }
        if (state==State.OUTTAKE) {
            state=State.DRIVING;

        }
    }
    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("Elbow Motor Encoder: ", elbow.getPosition(MotorUnit.TICKS));
        telemetry.addData("Extension Motor Encoder: ", extension.getPosition(MotorUnit.TICKS));

        processState();
        // Add logging if needed
        // Example:
        // telemetry.addData("Motor Encoder: ", motor.getPosition(MotorUnit.TICKS));
        // Handle states
        // Example:
//        switch (state) {
//            case ENABLED:
//                setPower(1.0);
//                break;
//            case DISABLED:
//                setPower(0.0);
//                break;
//        }

        // OR

//        if (state == State.ENABLED) {
//            setPower(1.0);
//        } else if (state == State.DISABLED) {
//            setPower(0.0);
//        }
    }
}
