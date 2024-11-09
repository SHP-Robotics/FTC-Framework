//package org.firstinspires.ftc.teamcode.subsystems;
//
//import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kLeftSlideName;
//import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kMaxHeight;
//import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kPixelHeight;
//import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kRightSlideName;
//import static org.firstinspires.ftc.teamcode.shplib.Constants.Arm.kSlideTolerance;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode.shplib.Constants;
//import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
//
//public class ArmSubsystem extends Subsystem {
//    private final DcMotorEx leftSlide;
//    private final DcMotorEx rightSlide;
//    private int slidePos;
//
//    public enum State {
//        BOTTOM(0),
//        EXTENDED(250),
//        AUTOBOTTOM(220),
//        MIDDLE(500),
//        MIDHIGH(1600),
//        HIGH(2100),
//        CLIMB(3300),
//        BOTTOMCLIMB(10),
//        FINISHCLIMB(200),
//        SAFETY(-860),
//        NOPOWER(0);
//
//        final double position;
//
//        State(double position) {
//            this.position = position;
//        }
//    }
//
//    private State state;
//
//    public ArmSubsystem(HardwareMap hardwareMap) {
//        slidePos = 0;
//
//        leftSlide = (DcMotorEx) hardwareMap.get(kLeftSlideName);
//        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        rightSlide = (DcMotorEx) hardwareMap.get(kRightSlideName);
//        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        setState(State.BOTTOM);
//    }
//
//    public void setState(State state) {
//        this.state = state;
//    }
//
//    public State getState() {
//        return state;
//    }
//
//    public double getDriveBias() {
//        double setpoint = this.state == State.EXTENDED ? this.slidePos: this.state.position;
//
//        if (Math.abs(setpoint - this.getSlidePosition()) < kSlideTolerance) {
//            return Constants.Drive.kMinimumBias;
//        }
//
//        return Constants.Drive.kMaximumBias;
//    }
//
//    public double getSlidePosition() {
//        return ((float)leftSlide.getCurrentPosition() + (float)rightSlide.getCurrentPosition()) / 2;
//    }
//
//    public void nextState(){
//        if(state == State.BOTTOM)
//            state = State.EXTENDED;
//        else if (state == State.EXTENDED) {
//            state = State.MIDDLE;
//        } else if(state==State.MIDDLE)
//            state = State.MIDHIGH;
//        else if(state == State.MIDHIGH)
//            state = State.HIGH;
//    }
//
//    public void setSlidePos(int slidePos) {
//        this.slidePos = slidePos;
//    }
//
//    public void incrementState(){
//        if(slidePos <= kMaxHeight-kPixelHeight)
//             slidePos += kPixelHeight;
//    }
//
//    public void decrementState(){
//        if(slidePos >= kPixelHeight)
//            slidePos -= kPixelHeight;
//    }
//
//
//    public void setPosition(double position){
//        rightSlide.setTargetPosition((int)position);
//        leftSlide.setTargetPosition((int)position);
//
//        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if (Math.abs(this.getSlidePosition() - position) < kSlideTolerance) {
//            rightSlide.setPower(0);
//            leftSlide.setPower(0);
//            return;
//        }
////
////        if (this.state == State.BOTTOM) {
////            if (this.getSlidePosition() < kSlideTolerance) {
////                rightSlide.setPower(0);
////                leftSlide.setPower(0);
////                return;
////            }
////        }
//
//        rightSlide.setPower(Constants.Arm.kRunPower);
//        leftSlide.setPower(Constants.Arm.kRunPower);
//    }
//
//    private void processState() {
//        if (this.state == State.NOPOWER){
//            this.rightSlide.setPower(0);
//            this.leftSlide.setPower(0);
//            return;
//        }
//
//        if (this.state == State.EXTENDED) {
//            this.setPosition(slidePos);
//            return;
//        }
//
//        this.setPosition(this.state.position);
//    }
//
//    @Override
//    public void periodic(Telemetry telemetry) {
//        processState();
//
//        telemetry.addData("State: ", state);
//        telemetry.addData("Left Slide Position: ", leftSlide.getCurrentPosition());
//        telemetry.addData("Right Slide Position: ", rightSlide.getCurrentPosition());
//        telemetry.addData("Left current", leftSlide.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Right current", rightSlide.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("avg current", (leftSlide.getCurrent(CurrentUnit.AMPS) + rightSlide.getCurrent(CurrentUnit.AMPS)) / 2);
////        telemetry.addData("Left Slide Velocity: ", leftSlide.getVelocity());
////        telemetry.addData("Right Slide Velocity: ", rightSlide.getVelocity());
//    }
//}
