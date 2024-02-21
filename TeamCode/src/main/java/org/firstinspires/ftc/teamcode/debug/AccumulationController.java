package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.util.ElapsedTime;

public class AccumulationController {
//    private double lastError = 0;
    private double lastTime = -1;
    private double lastOutput = 0;
    private final ElapsedTime elapsedTime;

    private final double kP;

    public AccumulationController(AccumulationControllerBuilder accumulationControllerBuilder) {
        this.kP = accumulationControllerBuilder.kP;

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    public double getLastOutput() {
        return this.lastOutput;
    }

    public double getOutput(double error) {
        if (lastTime == -1) {
            lastTime = elapsedTime.seconds();
        }

        double time = elapsedTime.seconds();
//        double deltaTime = time - lastTime;
        double deltaTime = 1;

        double output = lastOutput + (error * kP * deltaTime);

//        lastError = error;
        lastTime = time;
        lastOutput = output;

        return output;
    }

    public static class AccumulationControllerBuilder {
        private final double kP;

        public AccumulationControllerBuilder(double kP) {
            this.kP = kP;
        }

        public AccumulationController build() {
            return new AccumulationController(this);
        }
    }
}