package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PADController {
    private double lastError = 0;
    private double lastTime = -1;
    private double lastOutput;
    private ElapsedTime elapsedTime;

    private double kP;
    private double kD;

    public PADController(double kP, double kD) {
        this.kP = kP;
        this.kD = kD;
        elapsedTime.reset();
    }

    public double getOutput(double error) {
        if (lastTime == -1) {
            return 0;
        }

        double time = elapsedTime.seconds();
        double deltaTime = time - lastTime;

        double slope = (error - lastError) / deltaTime;

        double output = lastOutput + (error * kP * deltaTime) + (slope * kD * deltaTime);
        lastError = error;
        lastTime = time;
        lastOutput = output;
        return output;
    }

    public void setClampedLastOutput(double lastOutput) {
        this.lastOutput = lastOutput;
    }
}