package org.usfirst.frc.team2077.util;

import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.command.SelfDefinedCommand;

public class PIDTuner extends SelfDefinedCommand {

    private enum State{
        RUNNING, STOPPING
    }

    private final PIDTuneable module;
    private final double testDuration;
    private double[] setpoints;
    private double maxError = 0.0;

    private State state;

    private double bestError = Double.MAX_VALUE;
    private double bestP, bestI, bestD;

    private double timeSinceLastReset = 0.0;
    private double error = 0.0;

    private int setpointIndex = -1;

    public PIDTuner(PIDTuneable module, double[] setpoints, double duration){
        this.module = module;
        this.setpoints = setpoints;
        this.testDuration = duration;

        bestP = module.getP();
        bestI = module.getI();
        bestD = module.getD();

        state = State.STOPPING;

        for(double setpoint : setpoints) maxError += setpoint * duration;
    }

    public PIDTuner(PIDTuneable module, double setpoint, double duration){
        this(module, new double[]{setpoint}, duration);
    }

    @Override
    public void initialize() {
        state = State.STOPPING;
    }

    @Override
    public void execute(){
        double timeRunning = Clock.getSeconds() - timeSinceLastReset;

        switch (state){
            case RUNNING:
                module.tuningSet(setpoints[setpointIndex]);
                error += Math.abs(module.tuningGetError());

                if(timeRunning > testDuration){
                    state = State.STOPPING;
                }
                break;

            case STOPPING:
                module.tuningStop();
                if(module.tuningReady()){

                    timeSinceLastReset = Clock.getSeconds();
                    setpointIndex++;

                    if(setpointIndex >= setpoints.length){
                        randomWalk();

                        reset();
                        break;
                    }else{
                        state = State.RUNNING;
                    }
                }
        }
    }

    public void reset(){
        state = State.RUNNING;
        module.zeroIntegral();
        error = 0.0;
        timeSinceLastReset = Clock.getSeconds();
        setpointIndex = 0;
    }

    private void randomWalk(){

        if(error < bestError){
            bestError = error;
            bestP = module.getP();
            bestI = module.getI();
            bestD = module.getD();
        }


        double v = 5 * error / maxError;
        if(v > 1) v = 1;

        module.setP(vary(bestP, v));
        module.setI(vary(bestI, v));
        module.setI(vary(bestD, v));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted){
        module.setP(bestP);
        module.setI(bestI);
        module.setD(bestD);

        module.zeroIntegral();

        System.out.printf("P: %.8f\nI: %.8f\nD: %.8f\n", bestP, bestI, bestD);
    }

    public static double vary(double value, double variance){
        return value * (1 + variance * (Math.random() - 0.5));
    }

}
