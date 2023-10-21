package org.firstinspires.ftc.teamcode.shplib.hardware.drive.localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.hardware.sensors.SHPDistanceSensor;

/* TODO: decide whether to do noise/outlier filtering in this class (requires storing previous values)
 *   or in localizer class (compare current pose to estimated pose)
 * */

public class DistanceSensorOdometry {
    private final SHPDistanceSensor side, rear;
    private double prevX, prevY;


    public DistanceSensorOdometry(SHPDistanceSensor side, SHPDistanceSensor rear) {
        this.side = side;
        this.rear = rear;
//        this.prevX = -1;
//        this.prevY = -1;
    }

    private double getCorrectedAngle(double yaw) {
        return Math.toRadians(90) - Math.abs(yaw);
    }

    public double getXCorrected(double yaw) {
        return side.getDistance() * Math.cos(yaw);
    }

    public double getYCorrected(double yaw) {
        return rear.getDistance() * Math.sin(yaw);
    }

    public double getHeading(double yaw) {
        return Math.toRadians(90) - yaw;
    }

    public Pose2d getPoseEstimate(double yaw) {
        double x = getXCorrected(yaw);
        double y = getYCorrected(yaw);

        // maximum 5 inches of error
//        if ((prevX != -1 && prevY != -1) && (Math.abs(x - prevX) > 5.0 || Math.abs(y - prevY) > 5.0))
//            return null;
//
//        prevX = x;
//        prevY = y;
        return new Pose2d(x, y, getHeading(yaw));
    }

    public void log(Telemetry telemetry, double yaw) {
        telemetry.addData("Distance Measured (X): ", side.getDistance());
        telemetry.addData("Distance Measured (Y): ", rear.getDistance());
//        telemetry.addData("Corrected Angle (degs): ", Math.toDegrees(getCorrectedAngle(yaw)));
    }
}
