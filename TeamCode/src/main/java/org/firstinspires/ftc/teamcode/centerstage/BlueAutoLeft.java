package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(preselectTeleOp = "CenterStage Field Oriented")
public class BlueAutoLeft extends LinearOpMode {
    VisionSubsystem vision;

    public int location;

    @Override
    public void runOpMode() {
        //vision = new VisionSubsystem(hardwareMap,"blue");
        vision = new VisionSubsystem(hardwareMap,"blue");
        while (opModeInInit() && !isStopRequested()) {
            location = vision.getLocationBlue();
            telemetry.addData("Location: ", location);
            telemetry.update();
        }

        waitForStart();
    }
}
