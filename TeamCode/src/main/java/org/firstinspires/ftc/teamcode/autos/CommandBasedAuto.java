package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.DriveByCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.EncoderStraightDriveCommand;
import org.firstinspires.ftc.teamcode.commands.EncoderTurnDriveCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.shplib.controllers.FFController;
import org.firstinspires.ftc.teamcode.shplib.controllers.PositionPID;
import org.firstinspires.ftc.teamcode.shplib.hardware.drive.SHPRunnerMecanumDrive;

@Autonomous(preselectTeleOp = "CommandBasedTeleOp")
public class CommandBasedAuto extends BaseRobot {
    SHPRunnerMecanumDrive autoDrive;

    @Override

    public void init() {
        super.init();
        PositionPID pid = new PositionPID(0.15);
        pid.setErrorTolerance(100);
        autoDrive = new SHPRunnerMecanumDrive(hardwareMap, Constants.Drive.kMotorNames, 0.15, 0.0, 0.0,100);

    }

    @Override
    public void start() {
        super.start();

        CommandScheduler myCommand = CommandScheduler.getInstance();

        myCommand.scheduleCommand(
                new WaitCommand(2)
                        .then(new DriveByCommand(autoDrive,1000))
                        .then(new WaitCommand(2))
                        .then(new DriveByCommand(autoDrive,500,1500,1500, 500))

        );
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("auto drive at setpoint", autoDrive.atPositionSetpoint() ? "true" : "false");
    }
}
