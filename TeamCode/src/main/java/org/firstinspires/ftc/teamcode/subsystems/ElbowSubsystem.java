package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

public class ElbowSubsystem extends Subsystem {

  private DcMotorEx elbow;
  public enum State{
      UP(100),
      DOWN(10);
      final double position;
      State(double position){this.position=position;}
  }
  private State state;
  public ElbowSubsystem(HardwareMap hardwareMap){
      elbow = (DcMotorEx) hardwareMap.get("elbow");
      elbow.setDirection(DcMotorSimple.Direction.FORWARD);
      elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      //state = State.DOWN;
  }
  public void setState(State state){this.state = state;}
  public State getState(){return state;}

  public int getPosition(){
    return elbow.getCurrentPosition();
  }
  public DcMotorEx getElbow(){return elbow;}

  public void setPosition(double position){
    elbow.setTargetPosition((int) position);
    elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    elbow.setPower(1);
  }

  public void processState(){
      setPosition(this.state.position);
  }

  public void setPower(double power){
      elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      elbow.setPower(power);
  }

  @Override
  public void periodic(Telemetry telemetry){
     // processState();
      telemetry.addData("ElbowPos: ", elbow.getTargetPosition());
  }



}
