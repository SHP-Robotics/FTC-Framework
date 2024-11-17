package org.firstinspires.ftc.teamcode.autos.PurePursuit;

import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class BaseAuto extends BaseRobot {

    @Override
    public void init(){
        super.init();


        telemetry.update();
    }
    public void init_loop() {
        super.init_loop();

        telemetry.update();
    }

    CommandScheduler myCommand;
    @Override
    public void start(){
        super.start();
        myCommand = CommandScheduler.getInstance();
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.update();
    }
}
