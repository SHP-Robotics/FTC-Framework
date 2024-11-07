//package org.firstinspires.ftc.teamcode.autos.PurePursuit;
//
//import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
//import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
//import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
//
//public class BaseAuto extends BaseRobot {
//    public enum Side {
//        BLUE ("blue"),
//        RED ("red");
//
//        private final String color;
//
//        Side(String color) {
//            this.color = color;
//        }
//    }
//
//    public Side side = Side.BLUE;
//    private int location;
//
//    public int getLocation() {
//        return this.location;
//    }
//
//    @Override
//    public void init(){
//        super.init();
//
//        vision = new VisionSubsystem(hardwareMap, side.color);
//
//        if (side == Side.BLUE) {
//            location = vision.getLocationBlue();
//        } else {
//            location = vision.getLocationRed();
//        }
//
//        telemetry.addData("Location: ", location);
//        telemetry.update();
//    }
//    public void init_loop() {
//        super.init_loop();
//
//        if (side == Side.BLUE) {
//            location = vision.getLocationBlue();
//        } else {
//            location = vision.getLocationRed();
//        }
//
//        telemetry.addData("Location: ", location);
//        telemetry.update();
//    }
//
//    CommandScheduler myCommand;
//    @Override
//    public void start(){
//        super.start();
//        myCommand = CommandScheduler.getInstance();
//    }
//
//    @Override
//    public void loop() {
//        super.loop();
//        telemetry.update();
//    }
//}
