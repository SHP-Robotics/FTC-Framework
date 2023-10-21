package org.firstinspires.ftc.teamcode.shplib.hardware.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;
import org.firstinspires.ftc.teamcode.shplib.hardware.drive.localizer.Point;

public class SHPRunnerMecanumDrive extends SHPMecanumDrive {
    private final double[] setpoints;

    public SHPRunnerMecanumDrive(HardwareMap hardwareMap, String[] motorNames, double kP, double kI, double kD, double errorTolerance) {
        super(hardwareMap, motorNames);
        enablePositionPID(kP, kI, kD);
        setPositionErrorTolerance(errorTolerance);
        this.setpoints = getPositions(MotorUnit.TICKS);
        resetEncoders();
    }

    public void follow() {
        setPositions(setpoints);
    }

    public void translateTo(double leftFront, double leftRear, double rightFront, double rightRear) {
        setpoints[0] = leftFront;
        setpoints[1] = leftRear;
        setpoints[2] = rightFront;
        setpoints[3] = rightRear;
    }

    public void translateBy(double leftFront, double leftRear, double rightFront, double rightRear) {
        setpoints[0] += leftFront;
        setpoints[1] += leftRear;
        setpoints[2] += rightFront;
        setpoints[3] += rightRear;
    }

    public void translateBy(double[] positions) {
        for (int i = 0; i < setpoints.length; i++) {
            setpoints[i] += positions[i];
        }
    }

    public void resetEncoders() {
        for (SHPMotor motor : motors) {
            motor.resetEncoder();
        }
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("Left Front Position: ", motors[0].getPosition(MotorUnit.TICKS));
        telemetry.addData("Left Rear Position: ", motors[1].getPosition(MotorUnit.TICKS));
        telemetry.addData("Right Front Position: ", motors[2].getPosition(MotorUnit.TICKS));
        telemetry.addData("Right Rear Position: ", motors[3].getPosition(MotorUnit.TICKS));
        telemetry.addData("Runner At Setpoint: ", atPositionSetpoint() ? "True" : "False");
    }

}
