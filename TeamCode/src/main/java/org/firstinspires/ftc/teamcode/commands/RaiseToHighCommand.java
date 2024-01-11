//package org.firstinspires.ftc.teamcode.commands;
//
//import org.firstinspires.ftc.teamcode.shplib.commands.Command;
//import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
//
//public class RaiseToHighCommand extends Command {
//    private final ArmSubsystem arm;
//
//    public RaiseToHighCommand(ArmSubsystem arm) {
//        // You MUST call the parent class constructor and pass through any subsystems you use
//        super(arm);
//
//        this.arm = arm;
//    }
//
//    // Called once when the command is initially schedule
//    @Override
//    public void init() {
//        arm.setState(ArmSubsystem.State.HIGH);
//    }
//
//    // Specifies whether or not the command has finished
//    // Returning true causes execute() to be called once
//    @Override
//    public boolean isFinished() {
//        return arm.atSetpoint();
//    }
//}
