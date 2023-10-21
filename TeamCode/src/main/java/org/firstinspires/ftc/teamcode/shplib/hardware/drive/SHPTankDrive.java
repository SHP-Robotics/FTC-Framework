package org.firstinspires.ftc.teamcode.shplib.hardware.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class SHPTankDrive extends SHPFourWheelDrive {

    public SHPTankDrive(HardwareMap hardwareMap, String[] motorNames) {
        super(hardwareMap, motorNames);
    }

    public void tank(double leftPower, double rightPower) {
        double[] powers = {
                leftPower,
                leftPower,
                rightPower,
                rightPower
        };

        if (maxVelocity > 0) setAllVelocities(
                new double[]{
                        powers[0] * maxVelocity,
                        powers[1] * maxVelocity,
                        powers[2] * maxVelocity,
                        powers[3] * maxVelocity,
                }
        );
        else setAllPowers(powers);
    }

    public void arcade(double straight, double turn) {
        double[] powers = {
                straight + turn,
                straight + turn,
                straight - turn,
                straight - turn
        };

        if (maxVelocity > 0) setAllVelocities(
                new double[]{
                        powers[0] * maxVelocity,
                        powers[1] * maxVelocity,
                        powers[2] * maxVelocity,
                        powers[3] * maxVelocity,
                }
        );
        else setAllPowers(powers);
    }
}
