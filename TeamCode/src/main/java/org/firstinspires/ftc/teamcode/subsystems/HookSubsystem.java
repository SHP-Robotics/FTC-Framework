package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kHookLeftDisengaged;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kHookLeftEngaged;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kHookRightDisengaged;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kHookRightEngaged;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kHookServo1Name;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kHookServo2Name;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class HookSubsystem extends Subsystem {
    // Declare devices
    // Example:
    // private final SHPMotor motor;
    private final Servo hook1;
    private final Servo hook2;

    public enum State {
        // Define states
        // Example:
        ENGAGED, DISENGAGED
    }

    private State state;

    public HookSubsystem(HardwareMap hardwareMap) {
        hook1 = hardwareMap.get(Servo.class, kHookServo1Name);
        hook2 = hardwareMap.get(Servo.class, kHookServo2Name);

        // Set initial state
        // Example:
        state = State.DISENGAGED;
    }

    public void setState(State state) {
        this.state = state;
    }

    // Add control methods
    // Example:
    // private void setPower(double power) { motor.setPower(power); }

    @Override
    public void periodic(Telemetry telemetry) {
        // Add logging if needed
        // Example:
        // telemetry.addData("Motor Encoder: ", motor.getPosition(MotorUnit.TICKS));

        // Handle states
        // Example:
        switch (state) {
            case ENGAGED:
                hook1.setPosition(kHookLeftEngaged);
                hook2.setPosition(kHookRightEngaged);
                break;
            case DISENGAGED:
                hook1.setPosition(kHookLeftDisengaged);
                hook2.setPosition(kHookRightDisengaged);
                break;
        }
    }
}