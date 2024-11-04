package org.firstinspires.ftc.teamcode;
  
  // If Replit's expected package is "com.example.myproject" (replace with your actual package) // Update this line

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoMotors;

@TeleOp(name = "Competition Bot")
public class CompetitionBot extends OpMode {

  private DcMotor FrontLeft;
  private DcMotor FrontRight;
  private DcMotor BackLeft;
  private DcMotor BackRight;
  private DcMotor WormGear;
  private DcMotor Slide4;
  private Servos Servo1;
  
  @Override
  public void runOpMode() {
    FrontLeft = hardwareMap.get(DcMotor.class, "FL");
    FrontRight = hardwareMap.get(DcMotor.class, "FR");
    BackLeft = hardwareMap.get(DcMotor.class, "BL");
    BackRight = hardwareMap.get(DcMotor.class, "BR");
    WormGear = hardwareMap.get(DcMotor.class, "WormGear");
    Slide4 = hardwareMap.get(DcMotor.class, "VS4");
    Servo1 = hardwareMap.get(Servo.class, "Claw");
   
    // Setting power to drivetrain and long arm when system starts
     waitForStart();
      FrontLeft.setpower(DcMotor.getPower());
      FrontRight.setPower(DcMotor.getPower());
      BackLeft.setPower(DcMotor.getPower());
      BackRight.setPower(DcMotor.getPower());
      Slide4.setPower(DcMotor.getPower());
      Servo1.setPower(Servo.getpower());
      
    // Conditional statements for drivetrain to move with left joystick input forward and backward.
    if (gamepad1.left_stick_y > 0){
          FrontLeft.setDirection(DcMotor.Direction.FORWARD);
          FrontRight.setDirection(DcMotor.Direction.REVERSE);
          BackLeft.setDirection(DcMotor.Direction.FORWARD);
          BackRight.setdirection(DcMotor.Direction.REVERSE);
          telemetry.update();
      }
    if (gamepad1.left_stick_y < 0){
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        telemetry.update();
    }
    // Conditional statements for drivetrain to move with left joystick input left and right.
    if (gamepad1.left_stick_x > 0){
          FrontLeft.setDirection(DcMotor.Direction.FORWARD);
          FrontRight.setdirection(DcMotor.Direction.REVERSE);
          BackLeft.setdirection(DcMotor.Direction.FORWARD);
          BackRight.setdirection(DcMotor.Direction.REVERSE);
          telemetry.update();
      }
    if (gamepad1.left_stick_x < 0){
        FrontLeft.setdirection(DcMotor.Direction.REVERSE);
        FrontRight.setdirection(DcMotor.Direction.FORWARD);
        BackLeft.setdirection(DcMotor.Direction.REVERSE);
        BackRight.setdirection(DcMotor.Direction.FORWARD);
        telemetry.update();
    }
    
  }
}




/**
 * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
 * Comment Blocks show where to place Initialization code (runs once, after touching the
 * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
 * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
 * Stopped).
 */