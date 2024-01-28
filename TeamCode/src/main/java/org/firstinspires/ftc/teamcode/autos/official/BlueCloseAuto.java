package org.firstinspires.ftc.teamcode.autos.official;

import static org.firstinspires.ftc.teamcode.Constants.Arm.kSlideExtended;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.TestBaseRobot;
import org.firstinspires.ftc.teamcode.commands.IncrementUpArmCommand;
import org.firstinspires.ftc.teamcode.commands.LowerArmCommand;
import org.firstinspires.ftc.teamcode.commands.RaiseArmCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.AutonomousStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous
public class BlueCloseAuto extends TestBaseRobot {

    public enum State {
        PIXEL_1, PIXEL_2, PIXEL_3,
        LOCATION_1,
        DEPOSIT_1,
        TO_BACKDROP_1,

        LOCATION_2,
        DEPOSIT_2,
        TO_BACKDROP_2,

        LOCATION_3,
        DEPOSIT_3,
        TO_BACKDROP_3,
        DEPOSIT_TO_BACKDROP,

        TO_PARKING,
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
                    currentState = State.TO_BACKDROP_1;
                }
                break;
            case TO_BACKDROP_1:
                if (!sampleMecanumDrive.isBusy()) {
                    TrajectorySequence spikeMarkOneToBackdrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-15, 0, Math.toRadians(90)))
                            .lineToLinearHeading(new Pose2d(-17, -39.5, Math.toRadians(90)))
                            .build();

                    sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkOneToBackdrop);

                    currentState = State.DEPOSIT_TO_BACKDROP;

//                        currentState = State.ARM_1;
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
                    currentState = State.TO_BACKDROP_2;
                }
                break;
            case TO_BACKDROP_2:
                if (!sampleMecanumDrive.isBusy()) {
                    TrajectorySequence spikeMarkTwoToBackdrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-24, -39.5, Math.toRadians(90)))
                            .build();

                    sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkTwoToBackdrop);

                    currentState = State.DEPOSIT_TO_BACKDROP;
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
                    currentState = State.TO_BACKDROP_3;
                }
                break;

            case TO_BACKDROP_3:
                if (!sampleMecanumDrive.isBusy()) {

                    TrajectorySequence spikeMarkThreeToParking = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
//                                .lineToLinearHeading(new Pose2d(-15, 0, Math.toRadians(-90)))
                            .lineToLinearHeading(new Pose2d(-33, -39.5, Math.toRadians(90)))
                            .build();

                    sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkThreeToParking);

                    currentState = State.DEPOSIT_TO_BACKDROP;
                }

                break;

            // Common states
            case DEPOSIT_TO_BACKDROP:
                if (!sampleMecanumDrive.isBusy()) {
//                        sleep(1000);
                    myCommand.scheduleCommand(
                            new IncrementUpArmCommand(arm,wrist,elbow)
                                    .then(new RunCommand(()->{
                                        intake.setState(IntakeSubsystem.State.DEPOSIT2);
                                    }))
                                    .then(new WaitCommand(1.5))
                                    .then(new RunCommand(()->{intake.setState(IntakeSubsystem.State.STILL);}))
                                    .then(new LowerArmCommand(arm, wrist, elbow))
                                    .then(new RunCommand(()->{
                                        currentState = State.TO_PARKING;
                                    })));

//                        intake.setState(IntakeSubsystem.State.DEPOSIT2);
//                        sleep(1000);
//                        new LowerArmCommand(arm,wrist,elbow);
                    currentState = State.IDLE;
                }
                break;
            case TO_PARKING:
                if (!sampleMecanumDrive.isBusy()) {
                    TrajectorySequence backdropToPark = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                            .forward(5)
                            .lineToLinearHeading(new Pose2d(-2, -38, Math.toRadians(90)))
                            .back(5)
                            .build();
                    sampleMecanumDrive.followTrajectorySequenceAsync(backdropToPark);
                    currentState = State.IDLE;
                }
                break;
            case IDLE:
                break;
        }
    }
}
