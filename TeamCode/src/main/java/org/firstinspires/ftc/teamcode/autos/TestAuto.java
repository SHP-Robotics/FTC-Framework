//package org.firstinspires.ftc.teamcode.autos;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.BaseRobot;
//import org.firstinspires.ftc.teamcode.commands.EncoderStraightDriveCommand;
//import org.firstinspires.ftc.teamcode.commands.EncoderTurnDriveCommand;
//import org.firstinspires.ftc.teamcode.commands.EncoderTurnZeroCommand;
//import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
//import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
//import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
//
//@Autonomous(preselectTeleOp = "CommandBasedTeleOp")
//public class TestAuto extends BaseRobot {
//    //    SHPMecanumAutoDrive autoDrive;
//    DriveSubsystem drive;
//    VisionSubsystem vision;
//
//    public int location;
//
//    @Override
//    public void init() {
//        super.init();
////        PositionPID pid = new PositionPID(0.15);
////        pid.setErrorTolerance(100);
////        autoDrive = new SHPMecanumAutoDrive(hardwareMap, kMotorNames, 0.15, 0.0, 0.0);
////        autoDrive.enableFF(new FFController(0.01));
//        drive = new DriveSubsystem(hardwareMap);
//        vision = new VisionSubsystem(hardwareMap);
//        location = vision.getLocation();
//
//    }
//
//    public void init_loop() {
//        super.init_loop();
//        location = vision.getLocation();
//        telemetry.addData("Location: ", location);
//    }
//
//    @Override
//    public void start() {
//        super.start();
//
//        CommandScheduler myCommand = CommandScheduler.getInstance();
//
//        myCommand.scheduleCommand(
//                new WaitCommand(2)
//                        //.then(new RunCommand(() -> {location = vision.getLocation();}))
//                        .then(new EncoderStraightDriveCommand(drive,"forward",10,false))
//                        .then(new WaitCommand(1))
//                        .then(new EncoderTurnDriveCommand(drive,"cw",45))
//                        .then(new WaitCommand(1))
//                        .then(new EncoderTurnZeroCommand(drive))
//
//        );
//    }
//
//    @Override
//    public void loop() {
//        super.loop();
//        telemetry.addData("Location: ", location);
//    }
//}
