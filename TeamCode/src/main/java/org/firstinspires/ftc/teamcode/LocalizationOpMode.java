package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.tuners.LocalizationTuner;
@TeleOp

public class LocalizationOpMode extends LocalizationTuner {
    @Override
    public void setMecanumController(HardwareMap hardwareMap) {
        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
    }

    @Override
    public void setTracker(HardwareMap hardwareMap) {
        tracker = PestoFTCConfig.getTracker(hardwareMap);
    }


    @Override
    public void setTeleOpController(MecanumController mecanumController, DeterministicTracker deterministicTracker, HardwareMap hardwareMap) {
        teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, deterministicTracker, hardwareMap);
    }
}
