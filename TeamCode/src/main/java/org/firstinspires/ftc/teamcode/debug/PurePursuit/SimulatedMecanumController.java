package org.firstinspires.ftc.teamcode.debug.PurePursuit;

import static org.firstinspires.ftc.teamcode.debug.config.Constants.leftFrontDirection;
import static org.firstinspires.ftc.teamcode.debug.config.Constants.leftRearDirection;
import static org.firstinspires.ftc.teamcode.debug.config.Constants.rightFrontDirection;
import static org.firstinspires.ftc.teamcode.debug.config.Constants.rightRearDirection;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.debug.AccumulationControlledDcMotor;
import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

public class SimulatedMecanumController extends MecanumController {
    private double lastTime = -1;
    private final ElapsedTime elapsedTime;

    public SimulatedMecanumController(HardwareMap hardwareMap) {
        super(hardwareMap);

        this.leftFront = new AccumulationControlledDcMotor.AccumulationControlledDcMotorBuilder(this.leftFront)
                .setkP((Constants.leftFrontPower*0.7)/Constants.MAX_VELOCITY)
//                .setkP(1)
                .setGamma(0)
                .build();
        this.rightFront = new AccumulationControlledDcMotor.AccumulationControlledDcMotorBuilder(this.rightFront)
                .setkP((Constants.rightFrontPower*0.7)/Constants.MAX_VELOCITY)
//                .setkP(1)
                .setGamma(0)
                .build();
        this.leftRear = new AccumulationControlledDcMotor.AccumulationControlledDcMotorBuilder(this.leftRear)
                .setkP((Constants.leftRearPower*0.7)/Constants.MAX_VELOCITY)
//                .setkP(1)
                .setGamma(0)
                .build();
        this.rightRear = new AccumulationControlledDcMotor.AccumulationControlledDcMotorBuilder(this.rightRear)
                .setkP((Constants.rightRearPower*0.7)/Constants.MAX_VELOCITY)
//                .setkP(1)
                .setGamma(0)
                .build();

        // TODO: Set direction
        this.leftFront.setDirection(leftFrontDirection.inverted());
        this.rightFront.setDirection(rightFrontDirection.inverted());
        this.leftRear.setDirection(leftRearDirection.inverted());
        this.rightRear.setDirection(rightRearDirection);

        this.elapsedTime = new ElapsedTime();
        this.elapsedTime.reset();
    }

    public void driveParams(double x, double y, double r) {
        double leftFrontPower = y + x + r;
        double rightFrontPower = y - x - r;
        double leftRearPower = y - x + r;
        double rightRearPower = y + x - r;

        double[] velocities = new double[]{
                Math.abs(leftFrontPower),
                Math.abs(rightFrontPower),
                Math.abs(leftRearPower),
                Math.abs(rightRearPower)
        };

        double max = VelocityApproximator.getMaxVelocity(velocities);

//        if (max < 1) {
//            max = 1;
//        }
//
//        double min = VelocityApproximator.getBottleneck(velocities);
//
//        double multiplier = 1;
//        if (max != 0 && max - min > 0.2) {
//            multiplier = min/max;
//        }
//
//        this.leftFront.setPower(leftFrontPower * Constants.leftFrontPower * multiplier);
//        this.rightFront.setPower(rightFrontPower * Constants.rightFrontPower * multiplier);
//        this.leftRear.setPower(leftRearPower * Constants.leftRearPower * multiplier);
//        this.rightRear.setPower(rightRearPower * Constants.rightRearPower * multiplier);
//
        this.leftFront.setPower(leftFrontPower * Constants.leftFrontPower / max);
        this.rightFront.setPower(rightFrontPower * Constants.rightFrontPower / max);
        this.leftRear.setPower(leftRearPower * Constants.leftRearPower / max);
        this.rightRear.setPower(rightRearPower * Constants.rightRearPower / max);
    }

    public void driveFieldParams(double x, double y, double r, double gyro) {
        double xOriented = (Math.sin(gyro) * x) - (Math.cos(gyro) * y);
        double yOriented = (Math.sin(gyro) * y) + (Math.cos(gyro) * x);

        double leftFrontPower = yOriented + xOriented + r;
        double rightFrontPower = yOriented - xOriented - r;
        double leftRearPower = yOriented - xOriented + r;
        double rightRearPower = yOriented + xOriented - r;

        double[] velocities = new double[]{
                Math.abs(leftFrontPower),
                Math.abs(rightFrontPower),
                Math.abs(leftRearPower),
                Math.abs(rightRearPower)
        };

        double max = VelocityApproximator.getMaxVelocity(velocities);

//        if (max < 1) {
//            max = 1;
//        }

//        double min = VelocityApproximator.getBottleneck(velocities);
//
//        double multiplier = 1;
//        if (max != 0 && max - min > 0.2) {
//            multiplier = min/max;
//        }
//
//        this.leftFront.setPower(leftFrontPower * Constants.leftFrontPower * multiplier);
//        this.rightFront.setPower(rightFrontPower * Constants.rightFrontPower * multiplier);
//        this.leftRear.setPower(leftRearPower * Constants.leftRearPower * multiplier);
//        this.rightRear.setPower(rightRearPower * Constants.rightRearPower * multiplier);
//
        this.leftFront.setPower(leftFrontPower * Constants.leftFrontPower / max);
        this.rightFront.setPower(rightFrontPower * Constants.rightFrontPower / max);
        this.leftRear.setPower(leftRearPower * Constants.leftRearPower / max);
        this.rightRear.setPower(rightRearPower * Constants.rightRearPower / max);
    }

    public void simulateEncoders(Position2D deltaPosition) {
        if (this.lastTime != -1) {
            double[] approximatedVelocities = VelocityApproximator.getVelocities(deltaPosition, this.lastTime - this.elapsedTime.seconds());

            ((AccumulationControlledDcMotor)this.leftFront).setCurrentVelocity(approximatedVelocities[0]);
            ((AccumulationControlledDcMotor)this.rightFront).setCurrentVelocity(approximatedVelocities[1]);
            ((AccumulationControlledDcMotor)this.leftRear).setCurrentVelocity(approximatedVelocities[2]);
            ((AccumulationControlledDcMotor)this.rightRear).setCurrentVelocity(approximatedVelocities[3]);
        }
        this.lastTime = this.elapsedTime.seconds();
    }
}