package org.firstinspires.ftc.teamcode.PurePursuit;

import static org.firstinspires.ftc.teamcode.config.Constants.ODOMETRY_TICKS_PER_INCH;
import static org.firstinspires.ftc.teamcode.config.Constants.centerEncoderDirection;
import static org.firstinspires.ftc.teamcode.config.Constants.leftEncoderDirection;
import static org.firstinspires.ftc.teamcode.config.Constants.rightEncoderDirection;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.config.Constants;

public class PurePursuitFollower {
    private final Odometry leftOdometry;
    private final Odometry rightOdometry;
    private final Odometry centerOdometry;

    private Position2D robotVelocity;
    private Position2D currentPosition;

    private ElapsedTime elapsedTime;
    private double lastTime;

    public PurePursuitFollower(HardwareMap hardwareMap) {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: Set name (parameter 1)
        // TODO: Set ticks per inch (parameter 2)
        this.leftOdometry = new Odometry(
                (DcMotor)hardwareMap.get("left"),
                ODOMETRY_TICKS_PER_INCH);

        this.rightOdometry = new Odometry(
                (DcMotor)hardwareMap.get("right"),
                ODOMETRY_TICKS_PER_INCH);

        this.centerOdometry = new Odometry(
                (DcMotor)hardwareMap.get("center"),
                ODOMETRY_TICKS_PER_INCH);

        // TODO: Set direction
        this.leftOdometry.setDirection(leftEncoderDirection);
        this.rightOdometry.setDirection(rightEncoderDirection);
        this.centerOdometry.setDirection(centerEncoderDirection);

        this.leftOdometry.reset();
        this.rightOdometry.reset();
        this.centerOdometry.reset();

        this.currentPosition = new Position2D(0, 0, Math.PI/2);

        this.elapsedTime = new ElapsedTime();
        this.lastTime = elapsedTime.seconds();
    }

    public void reset() {
        this.currentPosition = new Position2D(0, 0, Math.PI/2);
    }

    public void resetTime() {
        this.lastTime = elapsedTime.seconds();
    }

    public void rotationTestingUpdateOdometry() {
        double lT = leftOdometry.getInchesTravelled();
        double cT = centerOdometry.getInchesTravelled();
        double rT = rightOdometry.getInchesTravelled();

        double distanceRotated = (lT - rT) / 2;
        double x = cT + (distanceRotated * Constants.FORWARD_OFFSET);
        double y = (lT + rT) / 2;
        double r = - (4 * distanceRotated) / Constants.ODOMETRY_WIDTH;

        double deltaTime = elapsedTime.seconds() - lastTime;
        lastTime = elapsedTime.seconds();
        this.robotVelocity = Position2D.multiply(new Position2D(x, y, r), 1/deltaTime);

        // TODO: check if r/2 helps or hinders
        // Should make all movement oriented between last and current position
        // because all movement occurred between last and current moment
        double headingRadians = getCurrentPosition().getHeadingRadians() + r/2;

        // TODO: replace with Position2D.rotate for simplicity

        // TODO: CHECK MATH
        double xOriented = (Math.sin(headingRadians) * x) + (Math.cos(headingRadians) * y);
        double yOriented = (Math.sin(headingRadians) * y) - (Math.cos(headingRadians) * x);

        currentPosition.add(new Position2D(
                xOriented,
                yOriented,
                r
        ), false);
    }

    public void updateOdometry() {
        double lT = leftOdometry.getInchesTravelled();
        double cT = centerOdometry.getInchesTravelled();
        double rT = rightOdometry.getInchesTravelled();

        double distanceRotated = (lT - rT) / 2;
        double x = cT + (distanceRotated * Constants.FORWARD_OFFSET);
        double y = (lT + rT) / 2;
        double r = - (4 * distanceRotated) / Constants.ODOMETRY_WIDTH;

        double deltaTime = elapsedTime.seconds() - lastTime;
        lastTime = elapsedTime.seconds();
        this.robotVelocity = Position2D.multiply(new Position2D(x, y, r), 1/deltaTime);

        // TODO: check if r/2 helps or hinders
        // Should make all movement oriented between last and current position
        // because all movement occurred between last and current moment
        double headingRadians = getCurrentPosition().getHeadingRadians() + r/2;

        // TODO: replace with Position2D.rotate for simplicity

        // TODO: CHECK MATH
        double xOriented = (Math.sin(headingRadians) * x) + (Math.cos(headingRadians) * y);
        double yOriented = (Math.sin(headingRadians) * y) - (Math.cos(headingRadians) * x);

        currentPosition.add(new Position2D(
                xOriented,
                yOriented,
                r
        ), true);
    }


    public Position2D getCurrentPosition() {
        return this.currentPosition;
    }

    public Position2D getRobotVelocity() {
        return robotVelocity;
    }
}
