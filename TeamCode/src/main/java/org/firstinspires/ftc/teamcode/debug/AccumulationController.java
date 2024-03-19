package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.util.ElapsedTime;

public class AccumulationController {
    private double lastError = 0;
    private double lastTime = -1;
    private double lastOutput;
    private ElapsedTime elapsedTime;

    private double kP;

    private boolean clamps;
    private double low;
    private double high;

    public AccumulationController(AccumulationControllerBuilder accumulationControllerBuilder) {
        this.kP = accumulationControllerBuilder.kP;

        this.clamps = accumulationControllerBuilder.clamps;
        this.low = accumulationControllerBuilder.low;
        this.high = accumulationControllerBuilder.high;

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    private double clamp(double x) {
        if (!this.clamps) {
            return x;
        }

        return Math.max(low, Math.min(high, x));
    }

    public double getOutput(double error) {
        if (lastTime == -1) {
            return 0;
        }

        double time = elapsedTime.seconds();
        double deltaTime = time - lastTime;

        double output = clamp(lastOutput - (error * kP * deltaTime));

        lastError = error;
        lastTime = time;
        lastOutput = output;

        return output;
    }

    public void setClampedLastOutput(double lastOutput) {
        this.lastOutput = lastOutput;
    }

    public static class AccumulationControllerBuilder {
        private final double kP;

        private boolean clamps = false;
        private double low = -1;
        private double high = 1;

        public AccumulationControllerBuilder(double kP) {
            this.kP = kP;
        }

        public AccumulationControllerBuilder setClampFunction(double low, double high) {
            this.clamps = true;
            this.low = low;
            this.high = high;
            return this;
        }

        public AccumulationController build() {
            return new AccumulationController(this);
        }
    }
}