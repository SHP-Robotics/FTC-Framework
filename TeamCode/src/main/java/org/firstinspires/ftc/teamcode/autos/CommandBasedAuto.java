package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.EncoderStraightDriveCommand;
import org.firstinspires.ftc.teamcode.commands.EncoderTurnDriveCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
//import org.firstinspires.ftc.teamcode.commands.DropConeCommand;

@Autonomous(preselectTeleOp = "CommandBasedTeleOp")
public class CommandBasedAuto extends BaseRobot {
//    SHPMecanumAutoDrive autoDrive;

    @Override
    public void init() {
        super.init();
//        PositionPID pid = new PositionPID(0.15);
//        pid.setErrorTolerance(100);
//        autoDrive = new SHPMecanumAutoDrive(hardwareMap, kMotorNames, 0.15, 0.0, 0.0);
//        autoDrive.enableFF(new FFController(0.01));
    }

    @Override
    public void start() {
        super.start();

        CommandScheduler myCommand = CommandScheduler.getInstance();

        myCommand.scheduleCommand(
                new WaitCommand(2)

                        .then(new WaitCommand(0.5))
                        .then(new EncoderStraightDriveCommand(drive,"forward",5,false))
                        .then(new DriveCommand(drive,-0.2, 0,0,4,false))
                        .then(new WaitCommand(5))
                        .then(new EncoderTurnDriveCommand(drive,"cw",90))
                        .then(new WaitCommand(5))
                        .then(new EncoderTurnDriveCommand(drive,"ccw",90))
        );
    }

    @Override
    public void loop() {
        super.loop();
//        telemetry.addData("auto drive at setpoint", autoDrive.atPositionSetpoint() ? "true" : "false");
    }
}
