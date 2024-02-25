package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DriveCommand extends Command {
    private final DriveSubsystem drive;
    private double startTime;
    private double endTime;
    double leftY; double leftX; double rightX; double time;
    boolean robot = true;

    public DriveCommand(DriveSubsystem drive, double leftY, double leftX, double rightX, double time, boolean robot) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(drive);
        this.drive = drive;
        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;
        this.time = time;
        this.robot = robot;
    }


    // Called once when the command is initially schedule

    public void init() {
        startTime = Clock.now();
        drive.mecanum(0,0,0);

    }

    // Called repeatedly until isFinished() returns true
    @Override
    public void execute() {
        if (!robot)
            drive.mecanum(leftY,leftX,rightX);
        else
            drive.mecanum(leftY,leftX,rightX);
    }

    // Called once after isFinished() returns true
    @Override
    public void end() {
        drive.mecanum(0,0,0);
    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, time);
    }
}