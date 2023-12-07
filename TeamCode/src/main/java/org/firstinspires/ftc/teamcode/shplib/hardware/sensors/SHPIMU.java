package org.firstinspires.ftc.teamcode.shplib.hardware.sensors;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SHPIMU {
    private final IMU imu;

    public SHPIMU(HardwareMap hardwareMap, RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
                  RevHubOrientationOnRobot.UsbFacingDirection usbDirection) {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                logoDirection,
                usbDirection));
        imu.initialize(parameters);
    }

    public double getYaw(AngleUnit unit) {
        return imu.getRobotYawPitchRollAngles().getYaw(unit);
    }
    public void resetYaw() {
        imu.resetYaw();
    }
}
