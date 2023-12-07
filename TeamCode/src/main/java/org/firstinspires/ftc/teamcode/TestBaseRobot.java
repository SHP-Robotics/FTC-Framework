package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.AdjustHolder;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CRWheel;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HookServo1;
import org.firstinspires.ftc.teamcode.subsystems.HookServo2;
import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PixelServo;
import org.firstinspires.ftc.teamcode.subsystems.PlaneServo;
import org.firstinspires.ftc.teamcode.subsystems.PracticeArmServo;
import org.firstinspires.ftc.teamcode.subsystems.SpinningIntake;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * Template created by Ayaan Govil on 8/21/2021.
 *
 * FTC Java Documentation: http://ftctechnh.github.io/ftc_app/doc/javadoc/index.html
 *
 * Helpful Shortcuts:
 * - Ctrl/Command + / = Comment/Uncomment line (can highlight multiple lines)
 * - Ctrl/Command + B = Go to declaration (for any variable, class, or method)
 * - Ctrl/Command + Alt/Option + L = Auto format code
 */

public class TestBaseRobot extends OpMode {
    // Declare subsystems and devices
    public DriveSubsystem drive;
    //public CRWheel crWheel;
    public PixelServo pixelServo;
    //public CameraOn vision;
    public VisionSubsystem vision;
    //    public VisionSubsystem vision;
    public PracticeArmServo elbow;
    public HookSubsystem hook;
    public ArmSubsystem arm;
    //public SpinningIntake spinningIntake;
    public AdjustHolder wrist;
    public PlaneServo planeServo;

    public IntakeSubsystem intake;
//    public SpinningIntake spinningIntake;
//    public CRWheel cWheel;
//    public Flap flap;
//    public ScoopSubsystem scoop;

    public double previousTime = 0;

    // Called when you press the init button
    @Override
    public void init() {
        // Configures universal clock and scheduler - DO NOT DELETE!
        configure();

        // Initialize your subsystems and devices
        drive = new DriveSubsystem(hardwareMap);
        //crWheel = new CRWheel(hardwareMap);
        //spinningIntake = new SpinningIntake(hardwareMap);
        //vision = new CameraOn(hardwareMap);
//        vision = new VisionSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap);
        elbow = new PracticeArmServo(hardwareMap);
//        spinningIntake = new SpinningIntake(hardwareMap);
//        cWheel = new CRWheel(hardwareMap);
//        flap = new Flap(hardwareMap);
//        scoop = new ScoopSubsystem(hardwareMap);
//        intake = new SHPMotor(hardwareMap, "intake");
        hook = new HookSubsystem(hardwareMap);
        pixelServo = new PixelServo(hardwareMap);
        wrist = new AdjustHolder(hardwareMap);
        planeServo = new PlaneServo(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);



    }

    // Called when you press the start button
    @Override
    public void start() {

    }

    // Called repeatedly while an OpMode is running
    @Override
    public void loop() {
        telemetry.addData("Loop Time (ms): ", Clock.elapsed(previousTime) * 1000);
        previousTime = Clock.now();

        // Handles all subsystem and command execution - DO NOT DELETE!
        try {
            CommandScheduler.getInstance().run();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    // Called when you press the stop button
    @Override
    public void stop() {
        // Flushes any cached subsystems and commands - DO NOT DELETE!
        CommandScheduler.resetInstance();
    }

    public void configure() {
        // Starts universal clock - DO NOT DELETE!
        Clock.start();
        // Assigns telemetry object for Subsystem.periodic - DO NOT DELETE!
        CommandScheduler.getInstance().setTelemetry(telemetry);
        // Turn on bulk reads to help optimize loop times
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
}
