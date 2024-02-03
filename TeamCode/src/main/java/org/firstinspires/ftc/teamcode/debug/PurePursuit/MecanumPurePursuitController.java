package org.firstinspires.ftc.teamcode.debug.PurePursuit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;

public class MecanumPurePursuitController extends MecanumController {
    private Odometry leftOdometry;
    private Odometry rightOdometry;
    private Odometry centerOdometry;

    private final double ODOMETRY_WIDTH;
    private final double MECANUM_WIDTH;

    private Position2D currentPosition;

    public MecanumPurePursuitController(DcMotor leftFront, DcMotor rightFront, DcMotor leftRear, DcMotor rightRear, IMU imu,
                                        Odometry leftOdometry, Odometry rightOdometry, Odometry centerOdometry, double MECANUM_WIDTH, double ODOMETRY_WIDTH) {
        super(leftFront, rightFront, leftRear, rightRear, imu);

        this.leftOdometry = leftOdometry;
        this.rightOdometry = rightOdometry;
        this.centerOdometry = centerOdometry;

        this.leftOdometry.reset();
        this.rightOdometry.reset();
        this.centerOdometry.reset();

        this.MECANUM_WIDTH = MECANUM_WIDTH;
        this.ODOMETRY_WIDTH = ODOMETRY_WIDTH;

        this.currentPosition = new Position2D(0, 0, 0);
    }

    public MecanumPurePursuitController(HardwareMap hardwareMap, double MECANUM_WIDTH, double ODOMETRY_WIDTH) {
        super(hardwareMap);

        // TODO: Set name (parameter 1)
        // TODO: Set wheel diameter inches (parameter 2)
        // TODO: Set ticks per revolution (parameter 3)
        this.leftOdometry = new Odometry(
                (DcMotor)hardwareMap.get("left"),
                48/25.4,
                2000);

        this.rightOdometry = new Odometry(
                (DcMotor)hardwareMap.get("right"),
                48/25.4,
                2000);

        this.centerOdometry = new Odometry(
                (DcMotor)hardwareMap.get("center"),
                48/25.4,
                2000);

        // TODO: Set direction
        this.leftOdometry.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightOdometry.setDirection(DcMotorSimple.Direction.FORWARD);
        this.centerOdometry.setDirection(DcMotorSimple.Direction.FORWARD);

        this.leftOdometry.reset();
        this.rightOdometry.reset();
        this.centerOdometry.reset();

        this.MECANUM_WIDTH = MECANUM_WIDTH;
        this.ODOMETRY_WIDTH = ODOMETRY_WIDTH;

        this.currentPosition = new Position2D(0, 0, 0);
    }

    public double getMecanumWidth() {
        return this.MECANUM_WIDTH;
    }

    private static double closestToZero(double x1, double x2) {
        if (Math.abs(x1) < Math.abs(x2)) {
            return x1;
        }
        return x2;
    }

    public void rotationTestingUpdateOdometry() {
        currentPosition.add(new Position2D(
                centerOdometry.getInchesTravelled() - (Math.abs(leftOdometry.getInchesTravelled() - rightOdometry.getInchesTravelled()) / (ODOMETRY_WIDTH)),
                closestToZero(leftOdometry.getInchesTravelled(), rightOdometry.getInchesTravelled()),
                Math.abs(leftOdometry.getInchesTravelled() - rightOdometry.getInchesTravelled()) / ODOMETRY_WIDTH
                ), false);

        leftOdometry.reset();
        rightOdometry.reset();
        centerOdometry.reset();
    }

    public void updateOdometry() {
        currentPosition.add(new Position2D(
                centerOdometry.getInchesTravelled() - (Math.abs(leftOdometry.getInchesTravelled() - rightOdometry.getInchesTravelled()) / (ODOMETRY_WIDTH)),
                closestToZero(leftOdometry.getInchesTravelled(), rightOdometry.getInchesTravelled()),
                Math.abs(leftOdometry.getInchesTravelled() - rightOdometry.getInchesTravelled()) / ODOMETRY_WIDTH
        ), true);

        leftOdometry.reset();
        rightOdometry.reset();
        centerOdometry.reset();
    }

    public Position2D getCurrentPosition() {
        return this.currentPosition;
    }
}
