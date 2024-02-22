package org.firstinspires.ftc.teamcode.autos.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitFollower;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitPath;

@Autonomous(preselectTeleOp = "Centerstage Field Oriented")
public class BlueSpike extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        this.side = Side.BLUE;

        PurePursuitFollower purePursuitFollower = new PurePursuitFollower(hardwareMap);
        MecanumController mecanumController = new MecanumController(hardwareMap);

        PurePursuitPath path;
        PurePursuitPath path1 = new PurePursuitPath.PurePursuitPathBuilder()
                .moveTo(new Position2D(0, 25))
                .turnTo(new Position2D(-3, 28))
                .build();
        PurePursuitPath path2 = new PurePursuitPath.PurePursuitPathBuilder()
                .moveTo(new Position2D(0, 35.5))
                .build();
        PurePursuitPath path3 = new PurePursuitPath.PurePursuitPathBuilder()
                .moveTo(new Position2D(12.5, 28))
                .build();

        super.runOpMode();
        switch (this.location) {
            case FAR:
                path = path1;
                break;
            case CLOSE:
                path = path3;
                break;
            default:
                path = path2;
                break;
        }
        path.followAsync(purePursuitFollower, mecanumController);

        while (opModeIsActive() && !isStopRequested()) {
            path.update();
        }

        mecanumController.deactivate();
    }
}
