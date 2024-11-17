package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.EncoderTurnDriveCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(preselectTeleOp = "ATestTeleop")
public class TimeBasedAuto extends BaseRobot {

    @Override
    public void init() {
        super.init();
        telemetry.update();
        drive.resetIMUAngle();

    }
    public void init_loop() {
        super.init_loop();

    }

    @Override
    public void start() {
        super.start();

        CommandScheduler myCommand = CommandScheduler.getInstance();

        myCommand.scheduleCommand(
                new WaitCommand(2)
                        .then(new DriveCommand(drive,0.2,0,0,0.2,false))
                        .then(new DriveCommand(drive,0,0.5,0,1,false))

        );
    }

    @Override
    public void loop() {
        super.loop();
//        telemetry.addData("Location: ",location);
//        telemetry.addData("auto drive at setpoint", autoDrive.atPositionSetpoint() ? "true" : "false");
    }
}
