package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.hardware.drive.SHPRunnerMecanumDrive;

public class DriveByCommand extends Command {
    private final SHPRunnerMecanumDrive drive;
    private final double[] positions;

    public DriveByCommand(SHPRunnerMecanumDrive drive, double leftFront, double leftRear, double rightFront, double rightRear) {
        this.drive = drive;
        this.positions = new double[]{
                leftFront,
                leftRear,
                rightFront,
                rightRear
        };
    }

    public DriveByCommand(SHPRunnerMecanumDrive drive, double straight) {
        this.drive = drive;
        this.positions = new double[]{
                straight,
                straight,
                straight,
                straight
        };
    }

    // Called once when the command is initially schedule
    @Override
    public void init() {
        drive.translateBy(positions);
    }

    @Override
    public void execute() {
        drive.follow();
    }

    @Override
    public void end() {
        drive.setAllPowers(0.0);
    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return drive.atPositionSetpoint();
    }
}
