package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.ParametricHeading;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSubsystem;

@Config
@Autonomous(name = "Test Auto - 0+4")
public class TestAuto extends LinearOpMode {
    private MecanumController mecanumController;
    private DeterministicTracker tracker;
    VerticalSubsystem vertical;
    PivotSubsystem pivot;
    RotateSubsystem rotate;
    HorizSubsystem horizontal;
    ClawSubsystem claw;
    PathContainer depositBlock1, getBlock2, depositBlock2;

    PathFollower pathFollower;

    private ElapsedTime elapsedTime;

    public PathFollower generatePathFollower(PathContainer pathContainer, double deceleration, double speed) {
        return new PathFollower.PathFollowerBuilder(mecanumController, tracker, pathContainer)
                .setEndpointPID(PestoFTCConfig.endpointPID)
                .setHeadingPID(PestoFTCConfig.headingPID)
                .setDeceleration(PestoFTCConfig.DECELERATION)
                .setSpeed(speed)
//                .setDecelerationFunction(PathFollower.SQUID_DECELERATION)
                //^^^ combats static friction
                // takes the square root of PID. PID controls drive speed
                // call this "SQUID"
                //.setCheckFinishedFunction()
                .setEndTolerance(0.4, Math.toRadians(2))
                .setEndVelocityTolerance(4)
                .setTimeAfterDeceleration(deceleration)
                .build();
    }

    @Override
    public void runOpMode() {
        CommandScheduler.resetInstance();
        PestoFTCConfig.configure();

        depositBlock1 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.03)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(0, 0),
                                        new Vector2D(-10, -27)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, Math.toRadians(45) //90
                        })
                )
                .build();

        getBlock2 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-10, -27),
                                        new Vector2D(-12, -20)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                Math.toRadians(45), Math.toRadians(150) //150
                        })
                )

                .build();

        depositBlock2 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-12, -20),
                                        new Vector2D(-6, -24)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                Math.toRadians(150), Math.toRadians(45)
                        })
                )

                .build();


        Clock.start();
        CommandScheduler.getInstance().setTelemetry(telemetry);

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        vertical = new VerticalSubsystem(hardwareMap);
        pivot = new PivotSubsystem(hardwareMap);
        rotate = new RotateSubsystem(hardwareMap);
        horizontal = new HorizSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);

        claw.close();
        pivot.processState(PivotSubsystem.State.DRIVING);

        //TODO THIS IS THE START

        waitForStart();

        //Prepare to deposit preload
        raiseArm();

        //drive to sub
        followPath(depositBlock1, 2, 0.6);

        //Deposit preload
        lowerArm();
        prepIntake();

        //Push Block 1
        followPath(getBlock2, 2, 0.6);

        //Grab Block 1
        finishIntake();
        raiseArm();

        followPath(depositBlock2, 2, 0.6);

        lowerArm();

    }

    public void loopOpMode() {
        try {
            CommandScheduler.getInstance().run();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        tracker.update();
        pathFollower.update();

        telemetry.addData("Loop Times", elapsedTime.milliseconds());
        telemetry.addData("x", tracker.getCurrentPosition().getX());
        telemetry.addData("y", tracker.getCurrentPosition().getY());
        telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
        elapsedTime.reset();
        telemetry.update();
    }

    public void updateCommands(){
        try {
            CommandScheduler.getInstance().run();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public void updateCommands(double sec){
        elapsedTime.reset();
        while (elapsedTime.seconds() < sec) {
            tracker.update();
            updateCommands();
            if (isStopRequested()) return;
        }
    }
    public void followPath(PathContainer path, double deceleration, double speed){
        if (isStopRequested()) return;

        pathFollower = generatePathFollower(path, deceleration, speed);

        while (opModeIsActive() && !isStopRequested() && !pathFollower.isCompleted()) {
            loopOpMode();
        }
    }

    /** Prepares the intake sample */
    public void prepIntake(){
        //prep intake
        pivot.setState(PivotSubsystem.State.PREPAREINTAKE);
        rotate.setState(RotateSubsystem.State.INTAKE);
        claw.open();
        horizontal.setState(HorizSubsystem.State.INTAKINGEXTENDED);
        updateCommands();
    }
    /** Grabs sample and returns to driving mode */
    public void finishIntake(){
        pivot.setState(PivotSubsystem.State.INTAKE);
        updateCommands(0.5);

        claw.close();
        updateCommands(0.5);

        rotate.setState(RotateSubsystem.State.NEUTRAL);
        pivot.setState(PivotSubsystem.State.DRIVING);
        horizontal.setState(HorizSubsystem.State.DRIVING);
        updateCommands(0.5); //removed wait
    }

    /** Raises the Vertical */
    public void raiseArm(){
        horizontal.setState(HorizSubsystem.State.DRIVING);
        pivot.setState(PivotSubsystem.State.OUTTAKE1);
        updateCommands(0.5);

        pivot.setState(PivotSubsystem.State.OUTTAKE2);
        rotate.setState(RotateSubsystem.State.DROPOFF);
        vertical.setDepositState(VerticalSubsystem.State.HIGHBUCKET);
        vertical.setState(VerticalSubsystem.State.DEPOSITING);
        updateCommands(1.0); //removed wait
    }

    /** Deposits, and lowers arm */
    public void lowerArm(){
        updateCommands(0.5);

        claw.open();
        pivot.setState(PivotSubsystem.State.DRIVING);
        updateCommands(0.5);

        claw.close();
        vertical.setState(VerticalSubsystem.State.BOTTOM);
        updateCommands(0.5); //removed wait
    }
}