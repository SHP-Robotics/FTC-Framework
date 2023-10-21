package org.firstinspires.ftc.teamcode.shplib.hardware.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class SHPMecanumDrive extends SHPFourWheelDrive {

    public SHPMecanumDrive(HardwareMap hardwareMap, String[] motorNames) {
        super(hardwareMap, motorNames);
    }

    public void mecanum(double leftY, double leftX, double rightX) {
        double[] powers = {
                leftY - leftX - rightX,
                leftY + leftX - rightX,
                leftY + leftX + rightX,
                leftY - leftX + rightX,
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

    public void tankanum(double leftY, double rightY, double rightX) {
        double[] powers = {
                leftY - rightX,
                leftY + rightX,
                rightY + rightX,
                rightY - rightX
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
