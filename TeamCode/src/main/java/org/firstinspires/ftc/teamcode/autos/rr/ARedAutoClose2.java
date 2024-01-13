package org.firstinspires.ftc.teamcode.autos.rr;

import static org.firstinspires.ftc.teamcode.Constants.Arm.kLeftSlideName;
import static org.firstinspires.ftc.teamcode.Constants.Arm.kRightSlideName;
import static org.firstinspires.ftc.teamcode.Constants.Intake.kAdjustHolder;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.EncoderTurnDriveCommand;
import org.firstinspires.ftc.teamcode.commands.LowerArmCommand;
import org.firstinspires.ftc.teamcode.commands.RaiseArmCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.AutonomousStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.AdjustHolder;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PixelServo;
import org.firstinspires.ftc.teamcode.subsystems.PracticeArmServo;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(preselectTeleOp = "ATestTeleOp")
public class ARedAutoClose2 extends LinearOpMode {
    public enum State {
        LOCATION_1,
        DEPOSIT_1,
        TO_BACKDROP_1,
        ARM_1,

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

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousStorage.autonomousType = AutonomousStorage.AutonomousType.RedAutoLeftRR;

        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);
        State currentState;
        int location = 1;

        //subsystems
//        ArmSubsystem arm = new ArmSubsystem(hardwareMap);
        PracticeArmServo elbow = new PracticeArmServo(hardwareMap);
        PixelServo pixelServo = new PixelServo(hardwareMap);
        AdjustHolder wrist = new AdjustHolder(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        DcMotor leftSlide = hardwareMap.get(DcMotor.class, kLeftSlideName);
        DcMotor rightSlide = hardwareMap.get(DcMotor.class, kRightSlideName);
        Servo adjustHolder = hardwareMap.get(Servo.class, kAdjustHolder);

        //vision
        //TODO: SWITCH PIPELINE LATER
        VisionSubsystem visionSubsystem = new VisionSubsystem(hardwareMap,"red");
        location = visionSubsystem.getLocationRed();

        telemetry.addLine("Trajectory Sequence Ready");
        telemetry.addData("Location: ", location);
        telemetry.update();
        while (opModeInInit() && !isStopRequested()) {
            location = visionSubsystem.getLocationRed();
            telemetry.addLine("Trajectory Sequence Ready");
            telemetry.addData("Location: ", location);
            telemetry.update();
            adjustHolder.setPosition(0.6);
        }

        waitForStart();

        if (isStopRequested()) return;

        switch (location) {
            case 1:
                Trajectory pixelToSpikeMarkOne = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-28.75, -2, Math.toRadians(90)))
                        .build();

                sampleMecanumDrive.followTrajectoryAsync(pixelToSpikeMarkOne);
                currentState = State.DEPOSIT_1;
                break;
            case 2:
                Trajectory pixelToSpikeMarkTwo = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-31, 0, Math.toRadians(0)))
                        .build();

                sampleMecanumDrive.followTrajectoryAsync(pixelToSpikeMarkTwo);
                currentState = State.DEPOSIT_2;
                break;
            default:
                Trajectory pixelToSpikeMarkThree = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-28.75, 8.5, Math.toRadians(-90)))
                        .build();

                sampleMecanumDrive.followTrajectoryAsync(pixelToSpikeMarkThree);
                currentState = State.DEPOSIT_3; //TODO: FOR TESTING
                break;
        }
        //TODO: CURRENT AUTO 2+0 RIGHT RED
        while (opModeIsActive() && !isStopRequested()) {
            sampleMecanumDrive.update();

            telemetry.addData("Current State", currentState);
            telemetry.update();

            switch (currentState) {

                // Path series 1
                case DEPOSIT_1:
                    if (!sampleMecanumDrive.isBusy()) {
                        Trajectory spikeMarkOneBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-28.75, 4, Math.toRadians(90)))
                                .build();
                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkOneBackingUp);
                        currentState = State.IDLE;
                    }
                    break;
                case TO_BACKDROP_1:
                    if (!sampleMecanumDrive.isBusy()) {
                        TrajectorySequence spikeMarkOneToBackdrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-45, 40, Math.toRadians(-90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkOneToBackdrop);

                        currentState = State.ARM_1;
                    }
                    break;
                case ARM_1:
                    if (!sampleMecanumDrive.isBusy()) {
                        leftSlide.setPower(-0.8);
//                        rightSlide.setPower(0.5);
                        sleep(500);
                        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        adjustHolder.setPosition(0.2);
                        sleep(5);

                        currentState = State.IDLE;
                    }
                    break;

                // Path series 2
                case DEPOSIT_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        Trajectory spikeMarkTwoBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-24, 0, Math.toRadians(0)))
                                .build();
                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkTwoBackingUp);
                        currentState = State.IDLE;
                    }
                    break;
                case TO_BACKDROP_2:
                    if (!sampleMecanumDrive.isBusy()) {
                        TrajectorySequence spikeMarkTwoToBackdrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-28, 40, Math.toRadians(-90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkTwoToBackdrop);

                        currentState = State.TO_PARKING;
                    }
                    break;

                // Path series 3
                case DEPOSIT_3:
                    if (!sampleMecanumDrive.isBusy()) {
                        Trajectory spikeMarkThreeBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-28.75, 0, Math.toRadians(-90)))
                                .build();
                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkThreeBackingUp);
                        currentState = State.IDLE;
                    }
                    break;

                case TO_BACKDROP_3:
                    if (!sampleMecanumDrive.isBusy()) {

                        TrajectorySequence spikeMarkThreeToParking = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-15, 0, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-20, 42, Math.toRadians(-90)))
                                .build();

                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkThreeToParking);

                        currentState = State.TO_PARKING;
                    }

                    break;

                // Common states
                case DEPOSIT_TO_BACKDROP:
                    if (!sampleMecanumDrive.isBusy()) {
                        sleep(1000);
//                        new RaiseArmCommand(arm,wrist,elbow,pixelServo);
                        sleep(1000);
//                        pixelServo.setState(PixelServo.State.OUT);
                        sleep(1000);
//                        new LowerArmCommand(arm,wrist,elbow);
                        currentState = State.TO_PARKING;
                    }
                    break;
                case TO_PARKING:
                    if (!sampleMecanumDrive.isBusy()) {
                        TrajectorySequence backdropToPark = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
                                .forward(5)
                                .lineToLinearHeading(new Pose2d(-2,45,Math.toRadians(-85)))
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
}