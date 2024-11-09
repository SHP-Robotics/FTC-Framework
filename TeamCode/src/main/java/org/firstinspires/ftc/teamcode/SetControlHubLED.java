package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@Disabled
@Config
@TeleOp
public class SetControlHubLED extends LinearOpMode {
    public static int color = 1;

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        LynxModule controlHub = hubs.get(0);
        for (LynxModule hub: hubs) {
            if (!hub.isParent()) {
                controlHub = hub;
                break;
            }
        }

        controlHub.setConstant(color);
        
        waitForStart();

        controlHub.setConstant(0);
    }
}
