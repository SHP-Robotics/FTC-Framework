package org.firstinspires.ftc.teamcode.teleops.experimental;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.debug.MecanumController;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

// Not currently in a usable state, but I think roadrunner would get impatient
// Not recommended: 4/10
@Disabled
@TeleOp()
public class SpeedControlledMacros extends LinearOpMode {
    enum Mode {
        DRIVER_CONTROL,
        AUTONOMOUS_INIT,
        AUTONOMOUS_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    public class MasterDcMotor extends DcMotorImplEx {
        private double scalar = 1;

        public MasterDcMotor(DcMotorController controller, int portNumber) {
            super(controller, portNumber);
        }

        public MasterDcMotor(DcMotor dcMotor) {
            super(dcMotor.getController(), dcMotor.getPortNumber());
            super.setDirection(dcMotor.getDirection());
        }

        public void setScalar(double scalar) {
            this.scalar = scalar;
        }

        public double getScalar() {
            return this.scalar;
        }

        public double clamp(double power) {
            return Math.min(Math.max(-1, power), 1);
        }

        public void setPower(double power) {
            super.setPower(clamp(power) * this.scalar);
        }
    }

    class SampleMecanumDriveCancellableSpeedControlled extends SampleMecanumDriveCancelable {
        public SampleMecanumDriveCancellableSpeedControlled(HardwareMap hardwareMap) {
            super(hardwareMap);
            this.leftFront = new MasterDcMotor(this.leftFront);
            this.leftRear = new MasterDcMotor(this.leftRear);
            this.rightFront = new MasterDcMotor(this.rightFront);
            this.rightRear = new MasterDcMotor(this.rightRear);
        }

        public void masterDrive(Gamepad gamepad) {
            if (this.leftFront instanceof MasterDcMotor) {
                ((MasterDcMotor) this.leftFront).setScalar(gamepad.left_stick_y);
            }
            if (this.leftRear instanceof MasterDcMotor) {
                ((MasterDcMotor) this.leftRear).setScalar(gamepad.left_stick_y);
            }
            if (this.rightFront instanceof MasterDcMotor) {
                ((MasterDcMotor) this.rightFront).setScalar(gamepad.left_stick_y);
            }
            if (this.rightRear instanceof MasterDcMotor) {
                ((MasterDcMotor) this.rightRear).setScalar(gamepad.left_stick_y);
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancellableSpeedControlled drive = new SampleMecanumDriveCancellableSpeedControlled(hardwareMap);
        MecanumController mecanumController = new MecanumController(hardwareMap);

        // We want to turn off velocity control for teleop
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO: implement pose storage in autonomous
        // drive.setPoseEstimate(PoseStorage.currentPose);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            switch (currentMode) {
                case DRIVER_CONTROL:

                    mecanumController.drive(gamepad1);

                    if (gamepad1.a) {
                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(new Pose2d(0, 0, 0))
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTONOMOUS_INIT;
                    }
                    break;
                case AUTONOMOUS_INIT:
                    if (gamepad1.x) {
                        drive.breakFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    if (gamepad1.b) {
                        drive.masterDrive(gamepad1);
                    }

                    if (!drive.isBusy()) {
                        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(Math.sqrt(75)*2.375, 23.75/2, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(0, 23.75, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                                .build();

                        drive.followTrajectorySequenceAsync(traj2);

                        currentMode = Mode.AUTONOMOUS_CONTROL;
                    }
                    break;
                case AUTONOMOUS_CONTROL:
                    if (gamepad1.x) {
                        drive.breakFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    if (gamepad1.b) {
                        drive.masterDrive(gamepad1);
                    }

                    if (!drive.isBusy()) {
                        currentMode = Mode.AUTONOMOUS_INIT;
                    }
                    break;
            }
        }
    }
}