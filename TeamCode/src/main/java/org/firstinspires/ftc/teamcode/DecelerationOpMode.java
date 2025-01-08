package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.tuners.DecelerationTuner;

@TeleOp
public class DecelerationOpMode extends DecelerationTuner {
    @Override
    public void setMecanumController(HardwareMap hardwareMap) {
        this.mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
    }

    @Override
    public void setTracker(HardwareMap hardwareMap) {
        this.tracker = PestoFTCConfig.getTracker(hardwareMap);
    }

    @Override
    public void setTeleOpController(MecanumController mecanumController, DeterministicTracker deterministicTracker, HardwareMap hardwareMap) {
        this.teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, deterministicTracker, hardwareMap);

        teleOpController.setSpeedController((gamepad) -> {
            return 0.6;
        });
    }
}
