package org.firstinspires.ftc.teamcode.commands;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class EncoderStraightDriveCommand extends Command {
    private final DriveSubsystem drive;
    private double startTime;
    private double xPos;
    private double yPos;
    double leftY; double leftX; double rightX; double time;

    double targetX, targetY;
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in

    public boolean robot;

    public EncoderStraightDriveCommand(DriveSubsystem drive, String direction, double distance,boolean robot) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(drive);
        this.drive = drive;
        if(direction.equals("forward"))
            this.leftY = -0.4;
        else
            this.leftY = 0.4;
        this.leftX = 0;
        this.rightX = 0;
        this.xPos = 0;
        this.yPos = inchesToEncoderTicks(distance);
        this.robot = robot;
        //System.out.println(xPos);

    }


    // Called once when the command is initially schedule

    public void init() {
        //drive.parallelEncoder.resetEncoder();
        //drive.perpendicularEncoder.resetEncoder();
        drive.perpendicularEncoder.resetEncoder();
        drive.parallelEncoder.resetEncoder();

    }
    public static double encoderTicksToInches(double ticks) {

        return WHEEL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public static double inchesToEncoderTicks(double inches) {
        double c = (((WHEEL_RADIUS*2)*Math.PI));
        double ticksPerInch = TICKS_PER_REV/c;
        return ticksPerInch * inches;
        //return inches/(WHEEL_RADIUS*2*TICKS_PER_REV*Math.PI);
    }
    public double getxPos() {
        return xPos;
    }
    public double getyPos() {
        return yPos;
    }


    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {
        if (!robot) {
            drive.mecanum(leftY,0,0);
        }
        else {
            drive.robotmecanum(leftY,0,0);
        }


    }

    double difference = 0;
    double percentage = 0;
    //@Override

    // Called once after isFinished() returns true
    @Override
    public void end() {
        //drive.parallelEncoder.resetEncoder();
        //drive.perpendicularEncoder.resetEncoder();
        drive.mecanum(0, 0, 0);
    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Math.abs(drive.parallelEncoder.getCurrentPosition())>Math.abs(yPos);

    }
}
