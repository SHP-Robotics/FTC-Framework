package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.EncoderStraightDriveCommand;
import org.firstinspires.ftc.teamcode.commands.EncoderTurnDriveCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(preselectTeleOp = "CommandBasedTeleOp")
public class RedAuto extends BaseRobot {
    //    SHPMecanumAutoDrive autoDrive;
    //DriveSubsystem drive;
    VisionSubsystem vision;

    public int location;

    @Override
    public void init() {
        super.init();
//        PositionPID pid = new PositionPID(0.15);
//        pid.setErrorTolerance(100);
//        autoDrive = new SHPMecanumAutoDrive(hardwareMap, kMotorNames, 0.15, 0.0, 0.0);
//        autoDrive.enableFF(new FFController(0.01));
        //drive = new DriveSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap,0);
        location = vision.getLocation();
        telemetry.addData("Location: ",location);
    }
    public void init_loop() {
        super.init_loop();
        location = vision.getLocation();
        telemetry.addData("Location: ", location);
    }

    @Override
    public void start() {
        super.start();
        int finalLoc = location;
        CommandScheduler myCommand = CommandScheduler.getInstance();

        myCommand.scheduleCommand(
                new WaitCommand(2)
                        //.then(new RunCommand(() -> {location = vision.getLocation();}))
                        //.then(new EncoderStraightDriveCommand(drive,"forward",10))
                        //.then(new EncoderTurnDriveCommand(drive,"ccw",45))
                        .then(new WaitCommand(0.5))
                        //.then(new RunCommand(() -> {location = vision.getLocation();}))

                        //.then(new DriveCommand(drive,0.025,0,0,3.5))
                        .then(new EncoderStraightDriveCommand(drive,"forward",23,false))
                        //.then(new EncoderTurnZeroCommand(drive))
                        .then(new WaitCommand(1))
                        .then(new RunCommand(() -> {
                            if (finalLoc == 1) {
                                myCommand.scheduleCommand(new EncoderTurnDriveCommand(drive,"ccw",75)
                                );
                            }
                            if (finalLoc == 3) {
                                myCommand.scheduleCommand(new EncoderTurnDriveCommand(drive,"cw",75)
                                );
                            }

                        }))
                        .then(new WaitCommand(5))
//                      zaj  .then(new EncoderStraightDriveCommand(drive,"backward",5,false))
                        .then(new RunCommand(() -> {
                            intake.setState(IntakeSubsystem.State.OUTTAKING);
                        }))
                        .then(new WaitCommand(1))
                        .then(new RunCommand(() -> {
                            intake.setState(IntakeSubsystem.State.PAUSED);
                        }))

                       /*
                        .then(new WaitCommand(0.5))
                        .then( new RunCommand(() -> {
                            intake.setState(IntakeSubsystem.State.PAUSED);
                        }))*/

//                        .then(new DriveByCommand(autoDrive, 3000, 3000, -3000, -3000))
//                        .then(new DriveByCommand(autoDrive, -2000, 2000, 2000, -2000))

        );
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("Location: ",location);
        telemetry.addData("Angle: " , drive.imu.getYaw(AngleUnit.DEGREES)*1);
//        telemetry.addData("auto drive at setpoint", autoDrive.atPositionSetpoint() ? "true" : "false");
    }
}