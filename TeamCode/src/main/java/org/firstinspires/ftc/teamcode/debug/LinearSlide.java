//package org.firstinspires.ftc.teamcode.debug;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class LinearSlide {
//    private DcMotor lift;
//
//    private double liftPower = 0.7;
//    private double staticPower = 0.3;
//
//    private boolean liftIsStatic = false;
//    private double staticPosition = 0;
//
//    private void init() {
//        lift.setDirection(DcMotor.Direction.REVERSE);
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//
//    public LinearSlide(HardwareMap hardwareMap, RuntimeType runtimeType) {
//        lift = hardwareMap.get(DcMotor.class, "lift");
//
//        init();
//
//        if (runtimeType != RuntimeType.AUTONOMOUS) {
//            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//    }
//
//    public void setLiftPower(double power) {
//        liftPower = power;
//    }
//
//    public void applyLiftBrakes() {
//        staticPosition = lift.getCurrentPosition();
//        lift.setTargetPosition((int) staticPosition);
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lift.setPower(staticPower);
//        liftIsStatic = true;
//    }
//
//    public void drive(Gamepad gamepad) {
//        if (gamepad.left_stick_y == 0) {
//            if (!liftIsStatic) {
//                applyLiftBrakes();
//            }
//        } else {
//            if (liftIsStatic) {
//                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                liftIsStatic = false;
//            }
//            lift.setPower(-gamepad.left_stick_y * liftPower);
//        }
//    }
//
//    public void deactivate() {
//        lift.setPower(0);
//    }
//
//    public void setLiftHeight(int inches, boolean wait) {
//        lift.setTargetPosition((int) (inches * Constants.LIFT_ENCODER_TICKS_PER_INCH));
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lift.setPower(liftPower);
//
//        if (wait) {
//            while (lift.isBusy()) {}
//        }
//    }
//
//    public void setClawHeight(int inches, boolean wait) {
//        inches -= Constants.CLAW_HEIGHT;
//        if (inches < 0) {
//            inches = 0;
//        }
//
//        lift.setTargetPosition((int) (inches * Constants.LIFT_ENCODER_TICKS_PER_INCH));
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lift.setPower(liftPower);
//
//        if (wait) {
//            while (lift.isBusy()) {}
//        }
//    }
//}
