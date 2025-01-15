package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.algorithms.PID;
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
@Autonomous(name = "3 + 0 SPECIMEN")
public class ThreeSpecimen extends LinearOpMode {
    private MecanumController mecanumController;
    private DeterministicTracker tracker;
    VerticalSubsystem vertical;
    PivotSubsystem pivot;
    RotateSubsystem rotate;
    HorizSubsystem horizontal;
    ClawSubsystem claw;
    PathContainer startToSub, pushBlock1, depositBlock1, subToBlock2, depositBlock2, park;

    PathFollower pathFollower;

    private ElapsedTime elapsedTime;
    public static double kp = 0.0;

    public PathFollower generatePathFollower(PathContainer pathContainer, double deceleration, double speed) {
        return new PathFollower.PathFollowerBuilder(mecanumController, tracker, pathContainer)
                .setEndpointPID(new PID(kp,0, 0)) //make 0.02
                .setHeadingPID(new PID(0.3, 0, 0))
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

        startToSub = new PathContainer.PathContainerBuilder()
                .setIncrement(0.1)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(0, 0),
                                        new Vector2D(5, -31)
                                }
                        ),
                        new ParametricHeading(new double[]{
                            0, 0 //90
                        })
                )
                .build();

        pushBlock1 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(5, -31),
                                        new Vector2D(-36, -25) //-35, -25
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-36, -25),
                                        new Vector2D(-36, -50)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        }))
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-36, -50),
                                        new Vector2D(-38, -50)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        }),
                        this::prepIntake

                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-38, -50),
                                        new Vector2D(-38, -6) //if 2 blocks (-38, -10)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )

                .build();
        //PUSH BLOCK 2
        /*
//        pushBlock2 = new PathContainer.PathContainerBuilder()
//                .setIncrement(0.03)
//                .addCurve(new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(-38, -10),
//                                        new Vector2D(-38, -47) //-40
//                                }
//                        )
//                )
//                .addCurve(new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(-38, -47),
//                                        new Vector2D(-47, -47)
//                                }
//                        )
//                )
//                .addCurve(new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(-47, -47),
////                                        new Vector2D(-26, -49),
//                                        new Vector2D(-47, -10)
//                                }
//                        )
//                )
//
//                .build();
*/

        depositBlock1 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-38, -6), //with 2 blocks (-47, 10)
                                        new Vector2D(5, -10)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(5, -10), //with 2 blocks (-47, 10)
                                        new Vector2D(14, -31)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .build();

        subToBlock2 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(14, -31), //with 2 blocks (-47, 10)
                                        new Vector2D(-5, -25)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-5, -25),
                                        new Vector2D(-30, -25)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-30, -20),
                                        new Vector2D(-35, -7)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .build();

        depositBlock2 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-35, -7), //with 2 blocks (-47, 10)
                                        new Vector2D(15, -5)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(15, -5), //with 2 blocks (-47, 10)
                                        new Vector2D(10, -31)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
                        })
                )
                .build();
        park = new PathContainer.PathContainerBuilder()
                .setIncrement(0.03)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(10, -31), //with 2 blocks (-47, 10)
                                        new Vector2D(-30, -6)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, 0 //90
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
        prepArm();
        raiseArm();

        //drive to sub
        followPath(startToSub, 1.0, 0.6);

        //Deposit preload
        lowerArm();

        //Push Block 1
        followPath(pushBlock1, 0.6, 0.6);

        //Grab Block 1
        finishIntake();
        prepArm();
        raiseArm();

        //Deposit Block 1
        followPath(depositBlock1, 0.6, 0.6);
        lowerArm();

        //Grab Block 2
        prepIntake();
        followPath(subToBlock2, 0.6, 0.6);
        finishIntake();
        prepArm();
        raiseArm();

        //Deposit block 2
        followPath(depositBlock2, 0.6, 0.6);
        lowerArm();

        followPath(park, 0.6, 0.6);

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

    /** Prepares the intake for wall, opens claw */
    public void prepIntake(){
        //prep intake
        pivot.setState(PivotSubsystem.State.PICKUP);
        horizontal.setState(HorizSubsystem.State.AUTOINTAKE);
        updateCommands();
        rotate.setState(RotateSubsystem.State.PICKUP);
        claw.open();
        updateCommands();
    }
    /** Grabs specimen and returns to driving mode */
    public void finishIntake(){
        updateCommands(1);

        claw.close();
        updateCommands(0.5);

        pivot.setState(PivotSubsystem.State.PICKUP2);
        rotate.setState(RotateSubsystem.State.NEUTRAL);
        updateCommands(0.5);
    }

    /** Raises the pivot */
    public void prepArm(){
        horizontal.setState(HorizSubsystem.State.DRIVING);
        pivot.setState(PivotSubsystem.State.OUTTAKE1);
        updateCommands(0.5);

        pivot.setState(PivotSubsystem.State.OUTTAKE2);
        rotate.setState(RotateSubsystem.State.DROPOFF);
        updateCommands();
    }
    /** Raises the Vertical */
    public void raiseArm(){
        vertical.setDepositState(VerticalSubsystem.State.HIGHBAR);
        vertical.setState(VerticalSubsystem.State.DEPOSITING);
        updateCommands(0.5);
    }

    /** Deposits, and lowers arm */
    public void lowerArm(){

        vertical.setState(VerticalSubsystem.State.DOWN);
        updateCommands(0.5);

        claw.open();
        vertical.setState(VerticalSubsystem.State.BOTTOM);
        pivot.setState(PivotSubsystem.State.DRIVING);
        updateCommands(0.5);
    }
}