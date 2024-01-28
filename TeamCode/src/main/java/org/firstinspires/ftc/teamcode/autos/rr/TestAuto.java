//package org.firstinspires.ftc.teamcode.autos.rr;
//
//import static org.firstinspires.ftc.teamcode.Constants.Arm.kLeftSlideName;
//import static org.firstinspires.ftc.teamcode.Constants.Arm.kRightSlideName;
//import static org.firstinspires.ftc.teamcode.Constants.Arm.kSlideD;
//import static org.firstinspires.ftc.teamcode.Constants.Arm.kSlideExtended;
//import static org.firstinspires.ftc.teamcode.Constants.Arm.kSlideG;
//import static org.firstinspires.ftc.teamcode.Constants.Arm.kSlideP;
//import static org.firstinspires.ftc.teamcode.Constants.Arm.kSlideS;
//import static org.firstinspires.ftc.teamcode.Constants.Arm.kSlideTolerance;
//import static org.firstinspires.ftc.teamcode.Constants.Intake.kAdjustHolder;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.roadrunner.drive.AutonomousStorage;
//import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.shplib.controllers.ElevatorFFController;
//import org.firstinspires.ftc.teamcode.shplib.controllers.PositionPID;
//import org.firstinspires.ftc.teamcode.shplib.hardware.SHPMotor;
//import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.PixelServo;
//import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
//
//@Autonomous(preselectTeleOp = "ATestTeleOp")
//public class TestAuto extends LinearOpMode {
//    public enum State {
//        LOCATION_1,
//        DEPOSIT_1,
//        TO_BACKDROP_1,
//        ARM_1,
//
//        LOCATION_2,
//        DEPOSIT_2,
//        TO_BACKDROP_2,
//
//        LOCATION_3,
//        DEPOSIT_3,
//        TO_BACKDROP_3,
//        DEPOSIT_TO_BACKDROP,
//
//        TO_PARKING,
//        IDLE
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        AutonomousStorage.autonomousType = AutonomousStorage.AutonomousType.RedAutoLeftRR;
//
//        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);
//        SHPMotor leftSlide;
//
////        ArmSubsystem armSubsystem = new ArmSubsystem(hardwareMap);
//
//        leftSlide = new SHPMotor(hardwareMap, kLeftSlideName);
////        leftSlide.reverseDirection();
//        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        leftSlide.enablePositionPID(new PositionPID(kSlideP, 0.0, kSlideD));
//        leftSlide.resetEncoder();
//        leftSlide.setPositionErrorTolerance(kSlideTolerance);
//        leftSlide.enableFF(new ElevatorFFController(kSlideS, kSlideG));
//
//
//        SHPMotor rightSlide;
//        rightSlide = new SHPMotor(hardwareMap, kRightSlideName);
//        rightSlide.reverseDirection();
//        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rightSlide.enablePositionPID(new PositionPID(kSlideP, 0.0, kSlideD));
//        rightSlide.resetEncoder();
//        rightSlide.setPositionErrorTolerance(kSlideTolerance);
//        rightSlide.enableFF(new ElevatorFFController(kSlideS, kSlideG));
//
//
//        State currentState;
//        int location = 0;
//
//        //vision
//        //TODO: SWITCH PIPELINE LATER
////        VisionSubsystem visionSubsystem = new VisionSubsystem(hardwareMap,"blue");
////        location = visionSubsystem.getLocationBlue();
//
//        telemetry.addLine("Trajectory Sequence Ready");
//        telemetry.addData("Location: ", location);
//        telemetry.update();
//        while (opModeInInit() && !isStopRequested()) {
////            location = visionSubsystem.getLocationBlue();
//            telemetry.addLine("Trajectory Sequence Ready");
//            telemetry.addData("Location: ", location);
//            telemetry.update();
//        }
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        switch (location) {
//            case 0:
//                leftSlide.setPosition(kSlideExtended);
//
//                rightSlide.setPosition(kSlideExtended);
//
//                currentState = State.IDLE;
//                return;
//            case 1:
//                Trajectory pixelToSpikeMarkOne = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
//                        .lineToLinearHeading(new Pose2d(-28.75, -2, Math.toRadians(90)))
//                        .build();
//
//                sampleMecanumDrive.followTrajectoryAsync(pixelToSpikeMarkOne);
//                currentState = State.DEPOSIT_1;
//                break;
//            case 2:
//                Trajectory pixelToSpikeMarkTwo = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
//                        .lineToLinearHeading(new Pose2d(-32, 0, Math.toRadians(0)))
//                        .build();
//
//                sampleMecanumDrive.followTrajectoryAsync(pixelToSpikeMarkTwo);
//                currentState = State.DEPOSIT_2;
//                break;
//            default:
//                TrajectorySequence pixelToSpikeMarkThree = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
//                        .lineToLinearHeading(new Pose2d(-28.75, 0, Math.toRadians(-90)))
//                        .lineToLinearHeading(new Pose2d(-28.75, 6.5, Math.toRadians(-90)))
//                        .build();
//
//                sampleMecanumDrive.followTrajectorySequenceAsync(pixelToSpikeMarkThree);
//                currentState = State.DEPOSIT_3; //TODO: FOR TESTING
//                break;
//        }
//        //TODO: CURRENT AUTO 2+0 RIGHT RED
//        while (opModeIsActive() && !isStopRequested()) {
//            sampleMecanumDrive.update();
//
//            telemetry.addData("Current State", currentState);
//            telemetry.update();
//
//            switch (currentState) {
//
//                // Path series 1
//                case DEPOSIT_1:
//                    if (!sampleMecanumDrive.isBusy()) {
//                        Trajectory spikeMarkOneBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
//                                .lineToLinearHeading(new Pose2d(-28.75, 4, Math.toRadians(90)))
//                                .build();
//                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkOneBackingUp);
//                        currentState = State.TO_BACKDROP_1;
//                    }
//                    break;
//                case TO_BACKDROP_1:
//                    if (!sampleMecanumDrive.isBusy()) {
//                        TrajectorySequence spikeMarkOneToBackdrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
//                                .lineToLinearHeading(new Pose2d(-15, 0, Math.toRadians(90)))
//                                .lineToLinearHeading(new Pose2d(-22, -38.5, Math.toRadians(90)))
//                                .build();
//
//                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkOneToBackdrop);
//
//                        currentState = State.DEPOSIT_TO_BACKDROP;
//
////                        currentState = State.ARM_1;
//                    }
//                    break;
//                case ARM_1:
//                    if (!sampleMecanumDrive.isBusy()) {
////                        leftSlide.setPower(-0.8);
//////                        rightSlide.setPower(0.5);
////                        sleep(500);
////                        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//////                        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////                        adjustHolder.setPosition(0.2);
////                        sleep(5);
////                        arm.setState(ArmSubsystem.State.EXTENDED);
//                        currentState = State.IDLE;
//                    }
//                    break;
//
//                // Path series 2
//                case DEPOSIT_2:
//                    if (!sampleMecanumDrive.isBusy()) {
//                        Trajectory spikeMarkTwoBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
//                                .lineToLinearHeading(new Pose2d(-24, 0, Math.toRadians(0)))
//                                .build();
//                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkTwoBackingUp);
//                        currentState = State.TO_BACKDROP_2;
//                    }
//                    break;
//                case TO_BACKDROP_2:
//                    if (!sampleMecanumDrive.isBusy()) {
//                        TrajectorySequence spikeMarkTwoToBackdrop = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
//                                .lineToLinearHeading(new Pose2d(-26, -38.5, Math.toRadians(90)))
//                                .build();
//
//                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkTwoToBackdrop);
//
//                        currentState = State.DEPOSIT_TO_BACKDROP;
//                    }
//                    break;
//
//                // Path series 3
//                case DEPOSIT_3:
//                    if (!sampleMecanumDrive.isBusy()) {
//                        Trajectory spikeMarkThreeBackingUp = sampleMecanumDrive.trajectoryBuilder(sampleMecanumDrive.getPoseEstimate())
//                                .lineToLinearHeading(new Pose2d(-28.75, 0, Math.toRadians(-90)))
//                                .build();
//                        sampleMecanumDrive.followTrajectoryAsync(spikeMarkThreeBackingUp);
//                        currentState = State.TO_BACKDROP_3;
//                    }
//                    break;
//
//                case TO_BACKDROP_3:
//                    if (!sampleMecanumDrive.isBusy()) {
//
//                        TrajectorySequence spikeMarkThreeToParking = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
////                                .lineToLinearHeading(new Pose2d(-15, 0, Math.toRadians(-90)))
//                                .turn(90)
//                                .lineToLinearHeading(new Pose2d(-30, -38.5, Math.toRadians(90)))
//                                .build();
//
//                        sampleMecanumDrive.followTrajectorySequenceAsync(spikeMarkThreeToParking);
//
//                        currentState = State.DEPOSIT_TO_BACKDROP;
//                    }
//
//                    break;
//
//                // Common states
//                case DEPOSIT_TO_BACKDROP:
//                    if (!sampleMecanumDrive.isBusy()) {
////                        sleep(1000);
////                        new RaiseArmCommand(arm,wrist,elbow,pixelServo);
//                        sleep(1000);
////                        intake.setState(IntakeSubsystem.State.DEPOSIT2);
//                        sleep(1000);
////                        new LowerArmCommand(arm,wrist,elbow);
//                        currentState = State.IDLE;
//                    }
//                    break;
//                case TO_PARKING:
//                    if (!sampleMecanumDrive.isBusy()) {
//                        TrajectorySequence backdropToPark = sampleMecanumDrive.trajectorySequenceBuilder(sampleMecanumDrive.getPoseEstimate())
//                                .forward(5)
//                                .lineToLinearHeading(new Pose2d(-2,45,Math.toRadians(-85)))
//                                .back(5)
//                                .build();
//                        sampleMecanumDrive.followTrajectorySequenceAsync(backdropToPark);
//                        currentState = State.IDLE;
//                    }
//                    break;
//                case IDLE:
//                    break;
//            }
//        }
//    }
//}