package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.Constants.Drive.kMotorNames;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.RaiseToHighCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

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

//        CommandScheduler.getInstance().scheduleCommand(
//                new RunCommand(() -> {
//                    arm.closeClaw();
//                })
//                        .then(new WaitCommand(0.5))
//                        .then(new RunCommand(() -> {
//                            arm.setState(ArmSubsystem.State.HUB);
//                        }))
//                        .then(new WaitCommand(0.5))
//                        .then(new DriveByCommand(autoDrive, 3000))
//                        .then(new RaiseToHighCommand(arm))
//                        .then(new DriveByCommand(autoDrive, 500))
//                        .then(new DropConeCommand(arm))
////                        .then(new DriveByCommand(autoDrive, 3000, 3000, -3000, -3000))
////                        .then(new DriveByCommand(autoDrive, -2000, 2000, 2000, -2000))
//
//        );
    }

    @Override
    public void loop() {
        super.loop();
//        telemetry.addData("auto drive at setpoint", autoDrive.atPositionSetpoint() ? "true" : "false");
    }
}
