package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class test extends LinearOpMode {
    /**
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
//        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
//        Tracker tracker = PestoFTCConfig.getTracker(hardwareMap);
//        TeleOpController teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);

        FourBarSubsystem fourBarSubsystem = new FourBarSubsystem(hardwareMap);
        fourBarSubsystem.setState(FourBarSubsystem.FourBarState.DOWN);
        fourBarSubsystem.update();

        waitForStart();
    }
}
