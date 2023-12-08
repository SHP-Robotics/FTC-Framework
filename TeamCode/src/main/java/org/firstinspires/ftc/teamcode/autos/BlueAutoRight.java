package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(preselectTeleOp = "CenterStage Field Oriented")
public class BlueAutoRight extends BaseRobot {
    //    SHPMecanumAutoDrive autoDrive;
    //DriveSubsystem drive;
    VisionSubsystem vision;

    public int location;

    @Override
    public void init() {
        super.init();
        vision = new VisionSubsystem(hardwareMap,"blue");
        location = vision.getLocationBlue();
        telemetry.addData("Location: ",location);
        telemetry.update();
    }
    public void init_loop() {
        super.init_loop();
        location = vision.getLocationBlue();
        telemetry.addData("Location: ", location);
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("Location: ",location);
//        telemetry.addData("auto drive at setpoint", autoDrive.atPositionSetpoint() ? "true" : "false");
    }
}
