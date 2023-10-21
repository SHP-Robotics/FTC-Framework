package org.firstinspires.ftc.teamcode.shplib.hardware.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.shplib.controllers.FFController;
import org.firstinspires.ftc.teamcode.shplib.controllers.GainSchedule;
import org.firstinspires.ftc.teamcode.shplib.controllers.PositionPID;
import org.firstinspires.ftc.teamcode.shplib.controllers.VelocityPID;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.shplib.hardware.units.MotorUnit;

import java.util.Arrays;
import java.util.Collections;

public class SHPFourWheelDrive {
    final SHPMotor[] motors;
    double maxVelocity = 0;

    PositionPID positionPID = null;

    public SHPFourWheelDrive(HardwareMap hardwareMap, String[] motorNames) {
        // 0 -> left front
        // 1 -> left rear
        // 2 -> right front
        // 3 -> right rear
        motors = new SHPMotor[4];
        for (int i = 0; i < motors.length; i++) {
            motors[i] = new SHPMotor(hardwareMap, motorNames[i]);
        }
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setMaxOutput(double maxOutput) {
        for (SHPMotor motor : motors) {
            motor.setMaxOutput(maxOutput);
        }
    }

    public void enableFF(FFController[] ffs) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].enableFF(ffs[i]);
        }
    }

    public void enablePositionPID(double kP, double kI, double kD) {
        for (SHPMotor motor : motors) {
            motor.enablePositionPID(new PositionPID(kP, kI, kD));
        }
    }

    public void scheduleGains(GainSchedule... schedules) {
        for (SHPMotor motor : motors) {
            motor.scheduleGains(schedules);
        }
    }

    public void setInitialPositions(MotorUnit unit) {
        for (SHPMotor motor : motors) {
            motor.setInitialPosition(unit);
        }
    }

    public void setPositionErrorTolerance(double errorTolerance) {
        for (SHPMotor motor : motors) {
            motor.setPositionErrorTolerance(errorTolerance);
        }
    }

    public void checkSchedules() {
        Integer[] indices = new Integer[]{
                motors[0].getScheduleIndex(),
                motors[1].getScheduleIndex(),
                motors[2].getScheduleIndex(),
                motors[3].getScheduleIndex(),
        };
        Integer maxIndex = Collections.max(Arrays.asList(indices));
        for (int i = 0; i < indices.length; i++) {
            if (indices[i] < maxIndex) {
                motors[i].setScheduleIndex(maxIndex);
            }
        }
    }

    public void setPositions(double position) {
        checkSchedules();
        for (SHPMotor motor : motors) {
            motor.setPosition(position);
        }
    }

    public void setPositions(double[] position) {
        checkSchedules();
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPosition(position[i]);
        }
    }

    public boolean atPositionSetpoint() {
        return motors[0].atPositionSetpoint()
                && motors[1].atPositionSetpoint()
                && motors[2].atPositionSetpoint()
                && motors[3].atPositionSetpoint();

    }

    public void enableVelocityPID(VelocityPID velocityPID, double maxVelocity) {
        for (SHPMotor motor : motors) {
            motor.enableVelocityPID(velocityPID);
        }
        this.maxVelocity = maxVelocity;
    }

    public void setAllPowers(double powers) {
        for (int i = 0; i < motors.length; i++) {
            setPower(i, powers);
        }
    }

    public void setAllPowers(double[] powers) {
        for (int i = 0; i < motors.length; i++) {
            setPower(i, powers[i]);
        }
    }

    public void setAllVelocities(double[] powers) {
        for (int i = 0; i < motors.length; i++) {
            setVelocity(i, powers[i]);
        }
    }

    public void setPower(int index, double power) {
        motors[index].setPower(power);
    }

    public void setVelocity(int index, double power) {
        motors[index].setVelocity(power);
    }

    public void reverseAll() {
        for (SHPMotor motor : motors) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void enableBuiltInVelocityControl() {
        for (SHPMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public double[] getPowers() {
        return new double[]{
                motors[0].getPower(),
                motors[1].getPower(),
                motors[2].getPower(),
                motors[3].getPower()
        };
    }

    public double[] getVelocities(MotorUnit unit) {
        return new double[]{
                motors[0].getVelocity(unit),
                motors[1].getVelocity(unit),
                motors[2].getVelocity(unit),
                motors[3].getVelocity(unit)
        };
    }

    public double[] getPositions(MotorUnit unit) {
        return new double[]{
                motors[0].getPosition(unit),
                motors[1].getPosition(unit),
                motors[2].getPosition(unit),
                motors[3].getPosition(unit)
        };
    }

    public double getPositionsAveraged(MotorUnit unit) {
        return (motors[0].getPosition(unit) +
                motors[1].getPosition(unit) +
                motors[2].getPosition(unit) +
                motors[3].getPosition(unit)) / 4;
    }
}
