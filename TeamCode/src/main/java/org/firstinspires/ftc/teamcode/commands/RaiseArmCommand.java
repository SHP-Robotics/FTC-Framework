package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.shplib.commands.Command;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.AdjustHolder;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PixelServo;
import org.firstinspires.ftc.teamcode.subsystems.PracticeArmServo;

public class RaiseArmCommand extends Command {
    private final ArmSubsystem arm;
    private final AdjustHolder wrist;
    private final PracticeArmServo elbow;

    private final PixelServo pixelServo;
    private  double startTime;

    public RaiseArmCommand(ArmSubsystem arm, AdjustHolder wrist, PracticeArmServo elbow, PixelServo pixelServo) {
        // You MUST call the parent class constructor and pass through any subsystems you use
        super(arm, wrist, elbow, pixelServo);

        this.arm = arm;
        this.wrist = wrist;
        this.elbow = elbow;
        this.pixelServo = pixelServo;

    }

    // Called once when the command is initially schedule
    @Override
    public void init() {
        startTime = Clock.now();

        arm.setState(ArmSubsystem.State.HIGH);
        wrist.setState(AdjustHolder.State.UP);
        elbow.setState(PracticeArmServo.State.UP);
    }
    @Override
    public void end() {
        pixelServo.setState(PixelServo.State.OUT);
    }

    // Specifies whether or not the command has finished
    // Returning true causes execute() to be called once
    @Override
    public boolean isFinished() {
        return Clock.hasElapsed(startTime, 2);
    }
}
