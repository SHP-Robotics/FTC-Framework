package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private enum SlideState {
        BOTTOM(0),
        MIDDLE(1500),
        TOP(1500),
        ;

        SlideState(int state) {
            this.state = state;
        }

        private final int state;

        public int getState() {
            return state;
        }
    }

    private void raiseSlide() {
        if (slideState == SlideState.BOTTOM) {
            slideState = SlideState.MIDDLE;
            return;
        }
        slideState = SlideState.TOP;
    }

    private void lowerSlide() {
        if (slideState == SlideState.TOP) {
            slideState = SlideState.MIDDLE;
            return;
        }
        slideState = SlideState.BOTTOM;
    }

    private enum WristState {
        BOTTOM(0.0),
        MIDDLE(0.57),
        ;

        WristState(double state) {
            this.state = state;
        }

        private final double state;

        public double getState() {
            return state;
        }
    }

    private void raiseWrist() {
        wristState = WristState.MIDDLE;
    }

    private void lowerWrist() {
        wristState = WristState.BOTTOM;
    }

    private enum ClawState {
        OPEN(0.7),
        CLOSED(0.0),
        ;

        ClawState(double state) {
            this.state = state;
        }

        private final double state;

        public double getState() {
            return state;
        }
    }

    private void openClaw() {
        clawState = ClawState.OPEN;
    }

    private void closeClaw() {
        clawState = ClawState.CLOSED;
    }

    private SlideState slideState;
    private WristState wristState;
    private ClawState clawState;

    private DcMotorEx slideLeft, slideRight;
    private Servo armLeft, armRight;
    private Servo claw;

    private final double slidePower = 1.0;

    public Arm(HardwareMap hardwareMap) {
        slideState = SlideState.BOTTOM;
        wristState = WristState.BOTTOM;
        clawState = ClawState.CLOSED;

        slideLeft = (DcMotorEx) hardwareMap.get("slideLeft");
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideRight = (DcMotorEx) hardwareMap.get("slideRight");
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLeft = (Servo) hardwareMap.get("arm1");
        armLeft.setDirection(Servo.Direction.FORWARD);

        armRight = (Servo) hardwareMap.get("arm2");
        armRight.setDirection(Servo.Direction.REVERSE);

        claw = (Servo) hardwareMap.get("claw");
    }

    private void updateSlides() {
        slideLeft.setTargetPosition(slideState.getState());
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(slidePower);

        slideRight.setTargetPosition(slideState.getState());
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setPower(slidePower);
    }

    private void updateArms() {
        armLeft.setPosition(wristState.getState());
        armRight.setPosition(wristState.getState());
    }

    private void updateClaw() {
        claw.setPosition(clawState.getState());
    }

    public void update(Gamepad gamepad1) {
        if (gamepad1.y) {
            raiseSlide();
        } else if (gamepad1.a) {
            lowerSlide();
        }

        if (gamepad1.dpad_down) {
            lowerWrist();
        } else if (gamepad1.dpad_up) {
            raiseWrist();
        }

        if (gamepad1.right_bumper) {
            closeClaw();
        }
        if (gamepad1.left_bumper) {
            openClaw();
        }

        updateSlides();
        updateArms();
        updateClaw();
    }
}
