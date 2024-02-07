package org.firstinspires.ftc.teamcode.debug.PurePursuit;

import static org.firstinspires.ftc.teamcode.debug.config.Constants.ODOMETRY_TICKS_PER_INCH;
import static org.firstinspires.ftc.teamcode.debug.config.Constants.centerEncoderDirection;
import static org.firstinspires.ftc.teamcode.debug.config.Constants.leftEncoderDirection;
import static org.firstinspires.ftc.teamcode.debug.config.Constants.rightEncoderDirection;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.config.Constants;

public class MecanumPurePursuitController extends MecanumController {
    private final Odometry leftOdometry;
    private final Odometry rightOdometry;
    private final Odometry centerOdometry;

    private Position2D currentPosition;

    public MecanumPurePursuitController(HardwareMap hardwareMap) {
        super(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: Set name (parameter 1)
        // TODO: Set ticks per inch (parameter 2)
        this.leftOdometry = new Odometry(
                (DcMotor)hardwareMap.get("leftRear"),
                ODOMETRY_TICKS_PER_INCH);

        this.rightOdometry = new Odometry(
                (DcMotor)hardwareMap.get("SpinningIntake"),
                ODOMETRY_TICKS_PER_INCH);

        this.centerOdometry = new Odometry(
                (DcMotor)hardwareMap.get("leftFront"),
                ODOMETRY_TICKS_PER_INCH);

        // TODO: Set direction
        this.leftOdometry.setDirection(leftEncoderDirection);
        this.rightOdometry.setDirection(rightEncoderDirection);
        this.centerOdometry.setDirection(centerEncoderDirection);

        this.leftOdometry.reset();
        this.rightOdometry.reset();
        this.centerOdometry.reset();

        this.currentPosition = new Position2D(0, 0, 0);
    }

    public void rotationTestingUpdateOdometry() {
        double lT = leftOdometry.getInchesTravelled();
        double cT = centerOdometry.getInchesTravelled();
        double rT = rightOdometry.getInchesTravelled();

        // rotation addition to x missing for tuning ForwardOffsetTuner
        double distanceRotated = (lT - rT) / 2;
        double x = cT + (distanceRotated * Constants.CIRCULAR_RATIO);
        double y = (lT + rT) / 2;
        double r = (2 * distanceRotated) / Constants.ODOMETRY_WIDTH;

        // removed x and y oriented to get center odometry dist

        // clamp is false for tuning OdometryWidthTuner
        currentPosition.add(new Position2D(
                x,
                y,
                r
        ), false);
    }

    public void updateOdometry() {
        double lT = leftOdometry.getInchesTravelled();
        double cT = centerOdometry.getInchesTravelled();
        double rT = rightOdometry.getInchesTravelled();

        double distanceRotated = (lT - rT) / 2;
        double x = cT + (distanceRotated * Constants.CIRCULAR_RATIO);
        double y = (lT + rT) / 2;
        double r = (2 * distanceRotated) / Constants.ODOMETRY_WIDTH;

        // TODO: check if r/2 helps or hinders
        // Should make all movement oriented between last and current position
        // because all movement occurred between last and current moment
        double headingRadians = -getCurrentPosition().getHeadingRadians() - r/2;

        // TODO: CHECK MATH
        double xOriented = (Math.cos(headingRadians) * x) - (Math.sin(headingRadians) * y);
        double yOriented = (Math.cos(headingRadians) * y) + (Math.sin(headingRadians) * x);

        currentPosition.add(new Position2D(
                xOriented,
                yOriented,
                r
        ), true);
    }

    public Position2D getCurrentPosition() {
        return this.currentPosition;
    }
}
