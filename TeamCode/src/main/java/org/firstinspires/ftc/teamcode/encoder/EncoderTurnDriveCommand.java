package org.firstinspires.ftc.teamcode.encoder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class EncoderTurnDriveCommand extends Command {
    private final DriveSubsystem drive;
    private double startTime;
    private double degrees;
    private double fullDegrees;
    public double initialHeading;
    double leftY; double leftX; double rightX;

    double targetX, targetY;
    String direction;
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in

    public EncoderTurnDriveCommand(DriveSubsystem drive, String direction, double degrees) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(drive);
        this.drive = drive;
        this.degrees = degrees;
        this.direction = direction;


    }


    // Called once when the command is initially schedule

    public void init() {

    }

    // Called repeatedly until isFinished() returns true
    //@Override

    double percentage = 0;
    double difference = 0;

    //@Override
    public void execute(){
        if (direction.equals("cw")) {
            drive.mecanum(0, 0, -0.25);
        }
        else {
            drive.mecanum(0, 0, 0.25);
        }
    }

    // Called once after isFinished() returns true
    @Override
    public void end() {
        drive.mecanum(0,0,0);
    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    //TODO: IMU IS WEIRD VALUES ARHGILSHDG BIOSAalifuhdlafbohub
    @Override
    public boolean isFinished() {
        if(direction.equals("cw")) //>0 means turning CW
            return (drive.imu.getYaw(AngleUnit.DEGREES)*-1)>degrees*0.98;
        else //turning CCW
            return (drive.imu.getYaw(AngleUnit.DEGREES))>degrees*0.98;

    }
}
