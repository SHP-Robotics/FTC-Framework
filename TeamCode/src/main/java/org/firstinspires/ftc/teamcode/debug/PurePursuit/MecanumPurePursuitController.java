package org.firstinspires.ftc.teamcode.debug.PurePursuit;

import static org.firstinspires.ftc.teamcode.debug.config.Constants.ODOMETRY_TICKS_PER_INCH;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

public class MecanumPurePursuitController extends MecanumController {
    private Odometry leftOdometry;
    private Odometry rightOdometry;
    private Odometry centerOdometry;

    private Position2D currentPosition;

    public MecanumPurePursuitController(DcMotor leftFront, DcMotor rightFront, DcMotor leftRear, DcMotor rightRear, IMU imu,
                                        Odometry leftOdometry, Odometry rightOdometry, Odometry centerOdometry) {
        super(leftFront, rightFront, leftRear, rightRear, imu);

        this.leftOdometry = leftOdometry;
        this.rightOdometry = rightOdometry;
        this.centerOdometry = centerOdometry;

        this.leftOdometry.reset();
        this.rightOdometry.reset();
        this.centerOdometry.reset();

        this.currentPosition = new Position2D(0, 0, 0);
    }

    public MecanumPurePursuitController(HardwareMap hardwareMap) {
        super(hardwareMap);

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
        this.leftOdometry.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightOdometry.setDirection(DcMotorSimple.Direction.FORWARD);
        this.centerOdometry.setDirection(DcMotorSimple.Direction.FORWARD);

        this.leftOdometry.reset();
        this.rightOdometry.reset();
        this.centerOdometry.reset();

        this.currentPosition = new Position2D(0, 0, 0);
    }

    private static double closestToZero(double x1, double x2) {
        if (Math.abs(x1) < Math.abs(x2)) {
            return x1;
        }
        return x2;
    }

    public void rotationTestingUpdateOdometry() {
        // rotation subtraction from x missing for tuning ForwardOffsetTuner
        double x = centerOdometry.getInchesTravelled();
        double y = (leftOdometry.getInchesTravelled() + rightOdometry.getInchesTravelled()) / 2;
        double r = (leftOdometry.getInchesTravelled() - rightOdometry.getInchesTravelled()) / Constants.ODOMETRY_WIDTH;

        // removed x and y oriented to get center odometry dist

        // clamp is false for tuning OdometryWidthTuner
        currentPosition.add(new Position2D(
                x,
                y,
                r
        ), false);

        leftOdometry.reset();
        rightOdometry.reset();
        centerOdometry.reset();
    }

    public void updateOdometry() {
        double distanceRotationallyTravelled = (leftOdometry.getInchesTravelled() - rightOdometry.getInchesTravelled())/2;
        double x = centerOdometry.getInchesTravelled() + (distanceRotationallyTravelled * Constants.CIRCULAR_RATIO);
        double y = (leftOdometry.getInchesTravelled() + rightOdometry.getInchesTravelled()) / 2;
        double r = (leftOdometry.getInchesTravelled() - rightOdometry.getInchesTravelled()) / Constants.ODOMETRY_WIDTH;

        double xOriented = (Math.cos(-getCurrentPosition().getHeadingRadians()) * x) - (Math.sin(-getCurrentPosition().getHeadingRadians()) * y);
        double yOriented = (Math.cos(-getCurrentPosition().getHeadingRadians()) * y) + (Math.sin(-getCurrentPosition().getHeadingRadians()) * x);

        currentPosition.add(new Position2D(
                xOriented,
                yOriented,
                r
        ), true);

        leftOdometry.reset();
        rightOdometry.reset();
        centerOdometry.reset();
    }

    public Position2D getCurrentPosition() {
        return this.currentPosition;
    }
}
