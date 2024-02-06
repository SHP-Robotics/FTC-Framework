package org.firstinspires.ftc.teamcode.autos.official;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.TestBaseRobot;
import org.firstinspires.ftc.teamcode.commands.IncrementUpArmCommand;
import org.firstinspires.ftc.teamcode.commands.LowerArmCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.AutonomousStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous
public class BlueBackupAuto extends TestBaseRobot {

    public enum State {
        PIXEL_1, PIXEL_2, PIXEL_3,
        DEPOSIT_1,

        DEPOSIT_2,

        DEPOSIT_3,

        IDLE
    }
    int location;
    SampleMecanumDrive sampleMecanumDrive;
    State currentState;
    //    VisionSubsystem visionSubsystem;
    @Override
    public void init(){
        super.init();
        AutonomousStorage.autonomousType = AutonomousStorage.AutonomousType.RedAutoLeftRR;
        sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);
        vision = new VisionSubsystem(hardwareMap, "blue");

        location = 2;
//        vision = new VisionSubsystem(hardwareMap,"blue");
        location = vision.getLocationBlue();

        telemetry.addLine("Trajectory Sequence Ready");
        telemetry.addData("Location: ", location);
        telemetry.update();

    }
    public void init_loop() {
        super.init_loop();
        location = vision.getLocationBlue();
        telemetry.addData("Location: ", location);

    }
    CommandScheduler myCommand;
    @Override
    public void start(){
        super.start();
//        location = vision.getLocationBlue();
        myCommand = CommandScheduler.getInstance();

        telemetry.addLine("Trajectory Sequence Ready");
        telemetry.addData("Location: ", location);
        telemetry.update();
        switch (location) {
            case 1:
                currentState = State.PIXEL_1;
                break;
            case 2:
                currentState = State.PIXEL_2;
                break;
            case 3:
                currentState = State.PIXEL_3;
                break;
        }

    }

    @Override
    public void loop(){
        super.loop();

        sampleMecanumDrive.update();

        telemetry.addData("Current State", currentState);
        telemetry.update();
//            switch (location) {
////                case 0:
////                    myCommand.scheduleCommand(new RaiseArmCommand(arm, wrist, elbow));
////                    currentState = State.IDLE;
////                    return;
//                case 1:
////                    Trajectory pixelToSpikeMarkOne = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
////                            .lineToLinearHeading(new Pose2d(-28.75, -2, Math.toRadians(90)))
////                            .build();
////                    myCommand.scheduleCommand(new RunCommand(()->{
////                        sampleMecanumDrive.followTrajectoryAsync(pixelToSpikeMarkOne);
////                    }));
//
//                    currentState = State.PIXEL_1;
//                    break;
//                case 2:
//                    Trajectory pixelToSpikeMarkTwo = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
//                            .lineToLinearHeading(new Pose2d(-32, 0, Math.toRadians(0)))
//                            .build();
//
//                    sampleMecanumDrive.followTrajectoryAsync(pixelToSpikeMarkTwo);
//                    currentState = State.IDLE;
//                    break;
//                default:
//                    TrajectorySequence pixelToSpikeMarkThree = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
//                            .lineToLinearHeading(new Pose2d(-28.75, 0, Math.toRadians(-90)))
//                            .lineToLinearHeading(new Pose2d(-28.75, 6.5, Math.toRadians(-90)))
//                            .build();
//
//                    sampleMecanumDrive.followTrajectorySequenceAsync(pixelToSpikeMarkThree);
//                    currentState = State.IDLE; //TODO: FOR TESTING
//                    break;
//            }

        switch (currentState) {

            // Path series 1
            case PIXEL_1:
                TrajectorySequence pixelToSpikeMarkOne = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-28.75, 0, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-28.75, -5, Math.toRadians(90)))
                        .build();
                myCommand.scheduleCommand(new RunCommand(()->{
                    sampleMecanumDrive.followTrajectorySequenceAsync(pixelToSpikeMarkOne);
                }));
                currentState = State.DEPOSIT_1;
                break;
            case DEPOSIT_1:
                if (!sampleMecanumDrive.isBusy()) {
                    Trajectory spikeMarkOneBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-28.75, 4, Math.toRadians(90)))
                            .build();
                    sampleMecanumDrive.followTrajectoryAsync(spikeMarkOneBackingUp);
                    currentState = State.IDLE;
                }
                break;


            // Path series 2
            case PIXEL_2:
                TrajectorySequence pixelToSpikeMarkTwo = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-32, 0, Math.toRadians(0)))
                        .build();
                myCommand.scheduleCommand(new RunCommand(()->{
                    sampleMecanumDrive.followTrajectorySequenceAsync(pixelToSpikeMarkTwo);
                }));
                currentState = State.DEPOSIT_2;
                break;
            case DEPOSIT_2:
                if (!sampleMecanumDrive.isBusy()) {
                    Trajectory spikeMarkTwoBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-24, 0, Math.toRadians(0)))
                            .build();
                    sampleMecanumDrive.followTrajectoryAsync(spikeMarkTwoBackingUp);
                    currentState = State.IDLE;
                }
                break;

            // Path series 3
            case PIXEL_3:
                TrajectorySequence pixelToSpikeMarkThree = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-28.75, 0, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-28.75, 6.5, Math.toRadians(-90)))
                        .build();
                myCommand.scheduleCommand(new RunCommand(()->{
                    sampleMecanumDrive.followTrajectorySequenceAsync(pixelToSpikeMarkThree);
                }));
                currentState = State.DEPOSIT_3;
                break;
            case DEPOSIT_3:
                if (!sampleMecanumDrive.isBusy()) {
                    Trajectory spikeMarkThreeBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-28.75, 0, Math.toRadians(-90)))
                            .build();
                    sampleMecanumDrive.followTrajectoryAsync(spikeMarkThreeBackingUp);
                    currentState = State.IDLE;
                }
                break;


            case IDLE:
                break;
        }
    }
}
