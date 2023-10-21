package org.firstinspires.ftc.teamcode.shplib.utility.filters;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

public class SlewRateLimiter {
    private final double rateLimit;
    private double prevVal;
    private double prevTime;

    /**
     * Creates a new SlewRateLimiter with the given rate limit and initial value.
     *
     * @param rateLimit    The rate-of-change limit, in units per second.
     * @param initialValue The initial value of the input.
     */
    public SlewRateLimiter(double rateLimit, double initialValue) {
        this.rateLimit = rateLimit;
        prevVal = initialValue;
        prevTime = Clock.now();
    }

    /**
     * Creates a new SlewRateLimiter with the given rate limit and an initial value of zero.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    public SlewRateLimiter(double rateLimit) {
        this(rateLimit, 0.0);
    }

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public double calculate(double input) {
        double currentTime = Clock.now();
        double elapsedTime = currentTime - prevTime;
        prevVal +=
                Range.clip(input - prevVal, -rateLimit * elapsedTime, rateLimit * elapsedTime);
        prevTime = currentTime;
        return prevVal;
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(double value) {
        prevVal = value;
        prevTime = Clock.now();
    }

    public void reset() {
        reset(0.0);
    }
}
