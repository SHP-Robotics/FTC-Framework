package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.Tracker;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    public enum SlideState {
        BOTTOM(0),
        MIDDLE(1500),
        TOP(1500),
        ;

        SlideState(int state) {
            this.state = state;
        }

        private int state;

        public int getState() {
            return state;
        }
    }
    public enum ArmState {
        BOTTOM(0.0),
        MIDDLE(0.5),
        ;

        ArmState(double state) {
            this.state = state;
        }

        private double state;

        public double getState() {
            return state;
        }
    }

    public SlideState incrementState(SlideState state) {
        if (state == SlideState.BOTTOM) {
            return slideState = SlideState.MIDDLE;
        }
        return slideState = SlideState.TOP;
    }

    public SlideState decrementState(SlideState state) {
        if (state == SlideState.TOP) {
            return slideState = SlideState.MIDDLE;
        }
        return slideState = SlideState.BOTTOM;
    }

    MecanumController mecanumController;
    Tracker tracker;
    TeleOpController teleOpController;
    DcMotorEx slideLeft, slideRight;
    Servo arm1, arm2, claw;

    SlideState slideState;
    ArmState armState;

    @Override
    public void init() {
        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);

        slideState = SlideState.BOTTOM;
        armState = ArmState.BOTTOM;

        slideLeft = (DcMotorEx) hardwareMap.get("slideLeft");
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        slideRight = (DcMotorEx) hardwareMap.get("slideRight");
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        arm1 = (Servo) hardwareMap.get("arm1");
        arm1.setDirection(Servo.Direction.FORWARD);
        arm2 = (Servo) hardwareMap.get("arm2");
        arm2.setDirection(Servo.Direction.REVERSE);
        claw = (Servo) hardwareMap.get("claw");
    }

    @Override
    public void loop() {
        telemetry.addData("arm position ", arm1.getPosition());
        tracker.updateOdometry();
        teleOpController.updateSpeed(gamepad1);
        teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if(gamepad1.y){
            slideLeft.setTargetPosition(incrementState(slideState).getState());
            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideLeft.setPower(1);

            slideRight.setTargetPosition(incrementState(slideState).getState());
            slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRight.setPower(1);
        }
        else if(gamepad1.a){
            slideLeft.setTargetPosition(decrementState(slideState).getState());
            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideLeft.setPower(1);

            slideRight.setTargetPosition(decrementState(slideState).getState());
            slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRight.setPower(1);
        }

        if(gamepad1.dpad_down){
            arm1.setPosition(0.57);
            arm2.setPosition(0.57);
        } else if (gamepad1.dpad_up) {
            arm1.setPosition(0.0);
            arm2.setPosition(0.0);
        }

        if(gamepad1.right_bumper){
            claw.setPosition(0.7);
        }
        if(gamepad1.left_bumper){
            claw.setPosition(0.0);
        }

    }
}