package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Intake.kSpinningIntakeName;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;

public class SpinningIntake extends Subsystem {
    // Declare devices
    // Example:
    private final SHPMotor spinTake;
    public enum State {
        // Define states
        // Example:
        IN, OUT, DISABLE
        // In .setPower(1)
        // Out .setPower(-1)
        // Disable .setPower(0)
    }

    private State state;

    public SpinningIntake(HardwareMap hardwareMap) {
        // Initialize devices
        // Example:
        spinTake = new SHPMotor(hardwareMap, kSpinningIntakeName);

        setState(State.DISABLE);
    }


    public void setState(State state) {
        this.state = state;
    }
    public State getState() {
        return state;
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
            case IN:
                spinTake.setPower(1.0);
                break;
            case OUT:
                spinTake.setPower(-10.0);
                break;
            case DISABLE:
                spinTake.setPower(0.0);
        }
    }
}
