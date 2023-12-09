package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.TestBaseRobot;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.EncoderTurnDriveCommand;
import org.firstinspires.ftc.teamcode.commands.LowerArmCommand;
import org.firstinspires.ftc.teamcode.commands.RaiseArmCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(preselectTeleOp = "ATestTeleop")
public class ABlueAuto extends TestBaseRobot {
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
        vision = new VisionSubsystem(hardwareMap,"blue");
        location = vision.getLocationBlue();
        telemetry.addData("Location: ",location);
        drive.resetIMUAngle();

    }
    public void init_loop() {
        super.init_loop();
        location = vision.getLocationBlue();
        telemetry.addData("Location: ", location);

    }

    @Override
    public void start() {
        super.start();

        CommandScheduler myCommand = CommandScheduler.getInstance();

        myCommand.scheduleCommand(
                new WaitCommand(2)
                        //.then(new RunCommand(() -> {location = vision.getLocation();}))
                        //.then(new EncoderStraightDriveCommand(drive,"forward",10))
                        //.then(new EncoderTurnDriveCommand(drive,"ccw",45))
                        //.then(new WaitCommand(0.5))
                        //.then(new RunCommand(() -> {location = vision.getLocation();}))

                        //.then(new DriveCommand(drive,0.025,0,0,3.5))
                        .then(new DriveCommand(drive,-0.3,0,0,3,false))
                        .then(new WaitCommand(1))
                        //.then(new EncoderTurnZeroCommand(drive))

                        //.then(new WaitCommand(1))
                        .then(new RunCommand(() -> {
                            if (location == 1) {
                                myCommand.scheduleCommand(
                                        new EncoderTurnDriveCommand(drive,"ccw",87)
                                                .then(new WaitCommand(3))
                                                .then(new DriveCommand(drive,-0.2,0,0,1,true))
                                                .then(new DriveCommand(drive,0.2,0,0,1.2,true))
                                                .then(new WaitCommand(1))
                                                .then(new DriveCommand(drive,0,-0.3,0,1.9,true))
                                                .then(new DriveCommand(drive,-0.4,0,0,2.75,true))
                                                .then(new RaiseArmCommand(arm,wrist,elbow,pixelServo))
                                                //.then(new RaiseArmCommand(arm,wrist,elbow,pixelServo))
                                                .then(new DriveCommand(drive,-0.2,0,0,1.3,true))
                                                .then(new RunCommand(() -> {
                                                    intake.setState(IntakeSubsystem.State.OUTTAKING);
                                                }))
                                                .then(new WaitCommand(1))
                                                .then(new RunCommand(() -> {
                                                    intake.setState(IntakeSubsystem.State.STILL);
                                                }))
                                                .then(new LowerArmCommand(arm,wrist,elbow))

                                );
                            }
                            else if (location == 3) {
                                myCommand.scheduleCommand(
                                        new EncoderTurnDriveCommand(drive,"cw",89)
                                                .then(new WaitCommand(3))
                                                .then(new DriveCommand(drive,-0.2,0,0,2.1,true))
                                                .then(new DriveCommand(drive,0.2,0,0,2,true))
                                                .then(new EncoderTurnDriveCommand(drive,"ccw",87))
                                                .then(new DriveCommand(drive,-0.425,0,0,2.7,true))
                                                .then(new DriveCommand(drive,0,0.325,0,1.4,true))
                                                .then(new RaiseArmCommand(arm,wrist,elbow,pixelServo))
                                                //.then(new RaiseArmCommand(arm,wrist,elbow,pixelServo))
                                                .then(new DriveCommand(drive,-0.2,0,0,1,true))
                                                .then(new RunCommand(() -> {
                                                    intake.setState(IntakeSubsystem.State.OUTTAKING);
                                                }))
                                                .then(new WaitCommand(1))
                                                .then(new RunCommand(() -> {
                                                    intake.setState(IntakeSubsystem.State.STILL);
                                                }))
                                                .then(new LowerArmCommand(arm,wrist,elbow))
                                );
                            }
                            else {
                                myCommand.scheduleCommand(
                                        new DriveCommand(drive,-0.2,0,0,1.35,true)
                                                .then(new DriveCommand(drive,0.2,0,0,1.45,true))
                                                .then(new EncoderTurnDriveCommand(drive,"ccw",87))
                                                .then(new DriveCommand(drive,-0.4,0,0,3,true))
                                                .then(new RaiseArmCommand(arm,wrist,elbow,pixelServo))
                                                //.then(new RaiseArmCommand(arm,wrist,elbow,pixelServo))
                                                .then(new DriveCommand(drive,-0.2,0,0,0.9,true))
                                                .then(new RunCommand(() -> {
                                                    intake.setState(IntakeSubsystem.State.OUTTAKING);
                                                }))
                                                .then(new WaitCommand(1))
                                                .then(new RunCommand(() -> {
                                                    intake.setState(IntakeSubsystem.State.STILL);
                                                }))
                                                .then(new LowerArmCommand(arm,wrist,elbow))
                                );
                            }

                        }))
                        //.then(new DriveCommand(drive,0.3,0,0,2,true))
                        .then(new WaitCommand(1))
                        .then(new DriveCommand(drive, 0.2, 0,0, 1, true))
                        .then(new DriveCommand(drive, 0,-0.2, 0,2, true))
                //.then(new DriveCommand(drive,0.5,0,0,1,true))


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
//        telemetry.addData("auto drive at setpoint", autoDrive.atPositionSetpoint() ? "true" : "false");
    }
}
