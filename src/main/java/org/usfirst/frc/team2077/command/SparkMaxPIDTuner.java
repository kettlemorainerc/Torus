package org.usfirst.frc.team2077.command;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.command.SelfDefinedCommand;
import org.usfirst.frc.team2077.subsystem.swerve.SwerveModule;
import org.usfirst.frc.team2077.util.SmartDashNumber;
import org.usfirst.frc.team2077.util.SmartDashString;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;

public class SparkMaxPIDTuner<Module> extends SelfDefinedCommand {

    public enum ErrorMethod{
        DIFFERENCE,
        ANGLE_DIFFERENCE
    }

    private final double walkPercent = 0.33;
    private final double entropyPercent = 0.25;

    private final List<Module> modules;
    private final List<SparkMaxPIDController> pids;
    private final BiConsumer<Module, Double> set;
    private final Function<Module, Double> get;
    private final Consumer<Module> save;
    private ArrayList<Tester> testers;

    private final JoystickButton endButton;

    private SmartDashString debug;
    private SmartDashNumber test;

    private Module bestModule;

    private ErrorMethod errorMethod;
    private double maxError;
    private double bestError = Double.MAX_VALUE;

    private double pBest = 0, iBest = 0;
    private double pGuess = 0, iGuess = 0;

    private double setpoint;
    private double testDuration;
    private double trial = 0;

    private double pt, dt;

    public SparkMaxPIDTuner(
            List<Module> modules, List<SparkMaxPIDController> pids,
            BiConsumer<Module, Double> set, Function<Module, Double> get, Consumer<Module> save,
            double pGuess, double iGuess,
            double setpoint, double testDuration,
            ErrorMethod errorMethod, double maxError,
            JoystickButton endButton
    ){
        this.modules = modules;
        this.pids = pids;
        this.set = set;
        this.get = get;
        this.save = save;
        this.endButton = endButton;

        pBest = pGuess;
        iBest = iGuess;

        this.pGuess = pGuess;
        this.iGuess = iGuess;

        if(pids.size() == 1){
            pBest = pids.get(0).getP();
            if(Math.abs(pBest) < 0.00001) pBest = pGuess;
            iBest = pids.get(0).getI();
            if(Math.abs(iBest) < 0.00001) iBest = iGuess;
        }

        System.out.println(pBest);

        this.setpoint = setpoint;
        this.testDuration = testDuration;

        this.errorMethod = errorMethod;
        this.maxError = maxError;

        testers = new ArrayList<>();
        for(int i = 0; i < modules.size(); i++){
            testers.add(
                new Tester(modules.get(i), pids.get(i))
            );
        }

        debug = new SmartDashString("AutoPID", "", true);

        start();
    }

    @Override
    public boolean isFinished() {
        boolean finished = endButton.getAsBoolean();
        return finished;
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
            trial++;
            return;
        }

        for(Tester t : testers){
            t.execute();
        }

//        testers.forEach(Tester::execute);
    }

    //TODO: rename this method
    private void walk(){
//        boolean singleModule = testers.size() == 1;

//        double bestError = Double.MAX_VALUE;

        double pBest = this.pBest, iBest = this.iBest;

        for(Tester tester : testers){
            double error = tester.getError();

            if(error < bestError){
                bestError = error;
                pBest = tester.getPID().getP();
                iBest = tester.getPID().getI();
                bestModule = tester.getModule();
//                for(Tester t : testers) save.accept(t.module);
            }
        }

//        System.out.println(bestError);

        debug.set(
            String.format("P: %.8f, I: %.8f Error: %.4f", pBest, iBest, bestError)
        );

        double pEntropy = 0.0;
        if(this.pBest != 0.0) pEntropy = pBest - this.pBest;

        double iEntropy = 0.0;
        if(this.iBest != 0.0) iEntropy = iBest - this.iBest;

        this.pBest = pBest;
        this.iBest = iBest;

        if(Math.abs(pBest) < 0.00000001 && Math.abs(iBest) < 0.00000001){
            this.pBest = pGuess;
            this.iBest = iGuess;
        }

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
        return true;
    }

    private void start(){
        testers.forEach(Tester::start);
    }

    @Override
    public void end(boolean interrupted) {
        for(Tester t : testers){
            t.getPID().setP(pBest);
            t.getPID().setI(iBest);

            save.accept(t.module);
        }
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
//            pid.setP(pBest * (0.5 * Math.random() + 0.75));
//            pid.setI(iBest * (0.5 * Math.random() + 0.75));
            pid.setI(iBest);
            pid.setP(pBest);
        }

        private void start(){
            startTime = Clock.getSeconds();
            error = 0;
            running = true;
        }

        private boolean hasEnded(){
            if(errorMethod == ErrorMethod.DIFFERENCE){
                return !running && Math.abs(getMes()) < 0.01;
            }else if(errorMethod == ErrorMethod.ANGLE_DIFFERENCE){
                return !running && Math.abs(SwerveModule.getAngleDifference(0, getMes())) < 0.25;
            }
            return false;
        }

        private void execute() {
            if (!running){
                end();
                return;
            }

            if(getElapsed() > testDuration || getError() > maxError){
                end();
                return;
            }

            set(setpoint);
        }

        private void end(){
            running = false;
//            System.out.println(getMes());
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
            double mod = 1;

            if(trial == 0){
                mod = 100;
            }

            switch(errorMethod){
                case DIFFERENCE:
                    diff = set - mes;
                    if(diff < 0){
                        mod = 500;
                    }
                    break;
                case ANGLE_DIFFERENCE:
                    diff = SwerveModule.getAngleDifference(set, mes);
                    if(diff < 0){
                        mod = 25;
                    }
                    break;
            }

//            System.out.printf("dt: %.8f, diff: %.8f ", dt, diff);


            error += Math.abs(dt * diff) * mod;
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
