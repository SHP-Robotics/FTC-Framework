package org.firstinspires.ftc.teamcode.debug.PurePursuit;

import static org.firstinspires.ftc.teamcode.debug.config.Constants.leftFrontDirection;
import static org.firstinspires.ftc.teamcode.debug.config.Constants.leftRearDirection;
import static org.firstinspires.ftc.teamcode.debug.config.Constants.rightFrontDirection;
import static org.firstinspires.ftc.teamcode.debug.config.Constants.rightRearDirection;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.debug.AccumulationControlledDcMotor;

public class SimulatedMecanumController extends MecanumController {
    public SimulatedMecanumController(HardwareMap hardwareMap) {
        super(hardwareMap);

        for (int i = 0; i < 4; i++) {
            this.motors[i] = new AccumulationControlledDcMotor.AccumulationControlledDcMotorBuilder(this.motors[i])
                    .setkP(0.0007)
                    .setGamma(0.1)
                    .build();
        }

        this.motors[0].setDirection(leftFrontDirection.inverted());
        this.motors[1].setDirection(rightFrontDirection.inverted());
        this.motors[2].setDirection(leftRearDirection.inverted());
        this.motors[3].setDirection(rightRearDirection.inverted());
    }

    public void simulateEncoders(double[] velocities) {
        ((AccumulationControlledDcMotor)this.motors[0]).setCurrentVelocity(velocities[0]);
        ((AccumulationControlledDcMotor)this.motors[1]).setCurrentVelocity(velocities[1]);
        ((AccumulationControlledDcMotor)this.motors[2]).setCurrentVelocity(velocities[2]);
        ((AccumulationControlledDcMotor)this.motors[3]).setCurrentVelocity(velocities[3]);
    }
}