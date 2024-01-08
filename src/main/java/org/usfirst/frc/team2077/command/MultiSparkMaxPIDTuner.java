package org.usfirst.frc.team2077.command;

import com.revrobotics.SparkMaxPIDController;
import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.command.SelfDefinedCommand;
import org.usfirst.frc.team2077.util.SmartDashNumber;
import org.usfirst.frc.team2077.util.SmartDashString;

import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.Function;

public class MultiSparkMaxPIDTuner<Module> extends SelfDefinedCommand {

    public enum ErrorMethod{
        DIFFERENCE,
        ANGLE_DIFFERENCE
    }

    private final double walkPercent = 0.33;
    private final double entropyPercent = 0.25;

    private ArrayList<Module> modules;
    private ArrayList<SparkMaxPIDController> pids;
    private BiConsumer<Module, Double> set;
    private Function<Module, Double> get;
    private ArrayList<Tester> testers;

    private SmartDashString debug;

    private Module bestModule;

    private ErrorMethod errorMethod;
    private double maxError;
    private double bestError = Double.MAX_VALUE;

    private double pBest = 0, iBest = 0;

    private double setpoint;
    private double testDuration;

    private double pt, dt;

    public MultiSparkMaxPIDTuner(
            ArrayList<Module> modules, ArrayList<SparkMaxPIDController> pids,
            BiConsumer<Module, Double> set, Function<Module, Double> get,
            double pGuess, double iGuess,
            double setpoint, double testDuration,
            ErrorMethod errorMethod, double maxError
    ){
        this.modules = modules;
        this.pids = pids;
        this.set = set;
        this.get = get;

        pBest = pGuess;
        iBest = iGuess;

        this.setpoint = setpoint;
        this.testDuration = testDuration;

        this.errorMethod = errorMethod;
        this.maxError = maxError;

        testers = new ArrayList<Tester>();
        for(int i = 0; i < modules.size(); i++){
            testers.add(
                new Tester(modules.get(i), pids.get(i))
            );
        }

        debug = new SmartDashString("AutoPID", "", true);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void initialize() {
        pt = Clock.getSeconds();
        start();
    }

    @Override
    public void execute() {
        double ct = Clock.getSeconds();
        dt = ct - pt; pt = ct;

        if(finishedTesting()){
            walk();
            start();
            return;
        }

        testers.forEach(Tester::execute);
    }

    //TODO: rename this method
    private void walk(){
        boolean singleModule = testers.size() == 1;

        double pBest = this.pBest, iBest = this.iBest;

        for(Tester tester : testers){
            double error = tester.getError();

            if(error < bestError){
                bestError = error;
                pBest = tester.getPID().getP();
                iBest = tester.getPID().getI();
                bestModule = tester.getModule();
            }
        }

        System.out.println(bestError);

        debug.set(
            String.format("P: %.8f, I: %.8f Error: %.4f", pBest, iBest, bestError)
        );

        double pEntropy = 0.0;
        if(this.pBest != 0.0) pEntropy = pBest - this.pBest;

        double iEntropy = 0.0;
        if(this.iBest != 0.0) iEntropy = iBest - this.iBest;

        this.pBest = pBest;
        this.iBest = iBest;

        for(Tester tester : testers){
            tester.getPID().setP(   walkValue(pBest, pEntropy)  );
            tester.getPID().setI(   walkValue(iBest, iEntropy)  );
        }
    }

    //TODO: rename this method
    private double walkValue(double value, double entropy){
        return value * (1 + walkPercent * (2 * Math.random() - 1)) + entropyPercent * entropy;
    }

    private boolean finishedTesting(){
        for(Tester tester : testers){
            if(!tester.hasEnded()) return false;
        }
        if(errorMethod == ErrorMethod.ANGLE_DIFFERENCE) setpoint += Math.random() * Math.PI / 2.0;
        return true;
    }

    private void start(){
        testers.forEach(Tester::start);
    }

    @Override
    public void end(boolean interrupted) {
        testers.forEach(Tester::end);
    }

    public class Tester{

        private Module module;
        private SparkMaxPIDController pid;

        private double startTime;
        private double error;
        private boolean running;

        public Tester(Module module, SparkMaxPIDController pid){
            this.module = module;
            this.pid = pid;

            initPID();
        }

        private void initPID(){
            //TODO: Determine if this should be randomised at init
            pid.setP(pBest * (0.5 * Math.random() + 0.75));
            pid.setI(iBest * (0.5 * Math.random() + 0.75));
        }

        private void start(){
            startTime = Clock.getSeconds();
            error = 0;
            running = true;
        }

        private boolean hasEnded(){
            switch(errorMethod){
                case DIFFERENCE:
                    return !running && Math.abs(getMes()) < 0.01;
                case ANGLE_DIFFERENCE:
                    return true;
            }
            return false;
        }

        private void execute(){
            if(!running) return;

            if(getElapsed() > testDuration || getError() > maxError){
                end();
                return;
            }

            set(setpoint);
        }

        private void end(){
            running = false;
            set(0.0);
        }

        private double getError(){
            if(!running){
                return error;
            }
            //If process is still running, add to the accumulative error.
            //Is integral between set and measured
            double diff = 0.0;
            double set = setpoint;
            double mes = getMes();

            switch(errorMethod){
                case DIFFERENCE:
                    diff = set - mes;
                    break;
                case ANGLE_DIFFERENCE:
                    diff = set - mes;
                    if(Math.abs(diff) > Math.PI) diff -= 2 * Math.PI * Math.signum(diff);
                    break;
            }

//            System.out.printf("dt: %.8f, diff: %.8f ", dt, diff);

            error += Math.abs(dt * diff);
//            System.out.println(error);
            return error;
        }

        private double getElapsed(){
            return Clock.getSeconds() - startTime;
        }

        public SparkMaxPIDController getPID() {
            return pid;
        }

        public double getMes(){
            return get.apply(module);
        }

        public void set(double v){
            set.accept(module, v);
        }

        public Module getModule() {
            return module;
        }
    }

}
