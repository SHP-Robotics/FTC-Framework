package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.debug.config.DrivingConfiguration;

public class WeightedMecanumController extends MecanumController {
    public WeightedMecanumController(HardwareMap hardwareMap) {
        super(hardwareMap);

        this.leftFront = new WeightedDcMotor(leftFront);
        this.rightFront = new WeightedDcMotor(rightFront);
        this.leftRear = new WeightedDcMotor(leftRear);
        this.rightRear = new WeightedDcMotor(rightRear);
    }

    public WeightedMecanumController(HardwareMap hardwareMap, SpeedController speedController) {
        super(hardwareMap, speedController);

        this.leftFront = new WeightedDcMotor(leftFront);
        this.rightFront = new WeightedDcMotor(rightFront);
        this.leftRear = new WeightedDcMotor(leftRear);
        this.rightRear = new WeightedDcMotor(rightRear);
    }

    public void setWeights(double leftFrontWeights, double rightFrontWeights, double leftRearWeights, double rightRearWeights) {
        if (this.leftFront instanceof WeightedDcMotor) {
            ((WeightedDcMotor) this.leftFront).setWeights(leftFrontWeights);
        }

        if (this.rightFront instanceof WeightedDcMotor) {
            ((WeightedDcMotor) this.rightFront).setWeights(rightFrontWeights);
        }

        if (this.leftRear instanceof WeightedDcMotor) {
            ((WeightedDcMotor) this.leftRear).setWeights(leftRearWeights);
        }

        if (this.rightRear instanceof WeightedDcMotor) {
            ((WeightedDcMotor) this.rightRear).setWeights(rightRearWeights);
        }
    }

    public void driveWeights(Gamepad gamepad) {
        double y = DrivingConfiguration.getValue(gamepad, DrivingConfiguration.STRAFE_UP);

        setWeights(y, y, y, y);
    }
}