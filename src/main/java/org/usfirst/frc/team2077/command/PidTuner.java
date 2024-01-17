package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.util.SmartDashString;

public abstract class PidTuner extends RepeatedCommand {
    protected static final double MAX_ERROR = 14;
    protected static final double ENTROPY_PERCENT = 1 / 4D;
    protected static final double RANDOM_PERCENT = 1 / 3D;

    protected final SmartDashString autoPid = new SmartDashString("Auto PID", "", true);

    // General State
    protected double currentP, currentI;
    protected double bestP, bestI;
    protected double minError = Double.MAX_VALUE;

    protected final double duration;

    // Current Test State
    protected double previousTick;

    protected boolean testRunning;
    protected double testStart, currentError;

    public PidTuner(double initialP, double initialI, double duration) {
        this.currentP = this.bestP = initialP;
        this.currentI = this.bestI = initialI;

        this.duration = duration;
    }

    @Override public void initialize() {
        super.initialize();
        this.previousTick = Clock.getSeconds();
    }

    @Override public void execute() {
        if(testRunning) {
            testTick();
        } else {
            iteratePAndI();
            nextTest();
        }
    }

    protected void nextTest() {
        testStart = Clock.getSeconds();
        currentError = 0;
        testRunning = true;
    }

    protected void iteratePAndI() {
        double bestP = this.bestP;
        double bestI = this.bestI;

        if(currentError < minError) {
            this.minError = this.currentError;
            bestI = this.currentI;
            bestP = this.currentP;
        }

        String debugInfo = String.format("[P=%.8f][I=%.8f][Err=%.4f]", bestP, bestI, minError);
        autoPid.set(debugInfo);

        this.currentP = randomize(bestP, entropyOf(bestP, this.bestP));
        this.currentI = randomize(bestI, entropyOf(bestI, this.bestI));

        this.bestP = bestP;
        this.bestI = bestI;
    }

    protected double randomize(double best, double entropy) {
        double random = RANDOM_PERCENT * (2 * Math.random() - 1);
        double entropyAdjustment = ENTROPY_PERCENT * entropy;

        return (best * random) + entropyAdjustment;
    }

    protected double entropyOf(double nextBest, double prevBest) {
        if(prevBest != 0) return nextBest - prevBest;

        return 0;
    }

    protected void testTick() {
        double currentTime = Clock.getSeconds();
        double testDuration = currentTime - this.testStart;

        double changeInTime = currentTime - this.previousTick;
        this.previousTick = currentTime;

        boolean testRanForDuration = testDuration > duration;
        boolean maxErrorPassed = currentError > MAX_ERROR;

        if(testRanForDuration || maxErrorPassed) {
            stopTest();

            if(isTestStopped()) {
                testRunning = false;
            }
        } else {
            test(changeInTime);
        }
    }

    protected abstract void test(double changeInTime);
    protected abstract void stopTest();
    protected abstract boolean isTestStopped();
}
