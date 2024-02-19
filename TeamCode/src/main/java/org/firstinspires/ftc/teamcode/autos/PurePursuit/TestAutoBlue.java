package org.firstinspires.ftc.teamcode.autos.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.Geometry.Position2D;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitFollower;
import org.firstinspires.ftc.teamcode.debug.PurePursuit.PurePursuitPath;

@Autonomous
public class TestAutoBlue extends BaseAuto {
    PurePursuitPath path1;
    PurePursuitPath path2;
    PurePursuitPath path3;

    PurePursuitPath path;

    PurePursuitFollower purePursuitFollower;
    MecanumController mecanumController;

    @Override
    public void init() {
        this.side = Side.BLUE;

        path1 = new PurePursuitPath.PurePursuitPathBuilder()
                .moveTo(new Position2D(-8, 0, Math.toRadians(90)))
                .moveTo(new Position2D(-8, 27, Math.toRadians(90)))
                .build();

        path2 = new PurePursuitPath.PurePursuitPathBuilder()
                .moveTo(new Position2D(0, 31, Math.toRadians(90)))
                .build();

        path3 = new PurePursuitPath.PurePursuitPathBuilder()
                .moveTo(new Position2D(5.875, 0, Math.toRadians(90)))
                .moveTo(new Position2D(5.875, 27, Math.toRadians(90)))
                .build();

        purePursuitFollower = new PurePursuitFollower(hardwareMap);
        mecanumController = new MecanumController(hardwareMap);

        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        if (this.getLocation() == 1) {
            path = path1;
        } else if (this.getLocation() == 2) {
            path = path2;
        } else {
            path = path3;
        }

        path.followAsync(purePursuitFollower, mecanumController);

        super.start();
    }

    @Override
    public void loop() {
        path.update();

        super.loop();
    }
}
