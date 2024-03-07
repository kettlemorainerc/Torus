package org.usfirst.frc.team2077.command;
//
//import com.revrobotics.SparkMaxPIDController;
//import org.usfirst.frc.team2077.common.Clock;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.command.SelfDefinedCommand;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.util.AutoPIable;

import java.util.ArrayList;
import java.util.List;

//import org.usfirst.frc.team2077.subsystem.swerve.SwerveModule;
//import org.usfirst.frc.team2077.util.SmartDashNumber;
import org.usfirst.frc.team2077.util.SmartDashString;
//
//import java.util.ArrayList;
//import java.util.List;
//import java.util.function.BiConsumer;
//import java.util.function.Consumer;
//import java.util.function.Function;
//
//public class SparkMaxPIDTuner<Module> extends SelfDefinedCommand {
public class AutoPITuner extends SelfDefinedCommand {

    public enum ErrorMethod{
        DIFFERENCE,
        ANGLE_DIFFERENCE
    }
//
    private final double walkPercent = 0.45;
    private final double entropyPercent = 0.25;

    private final JoystickButton endButton;

    private final List<AutoPIable> modules;
    private final ArrayList<Tester> testers;

    private double bestError = Double.MAX_VALUE;

    private double pBest, iBest;

    private double setpoint, testDuration;
    private double trial = 0;

    private double pt, dt;

    private SmartDashString debug;

    public AutoPITuner(
        List<AutoPIable> modules,
        double setpoint, double testDuration,
        JoystickButton endButton
    ){
        this.modules = modules;
        this.endButton = endButton;

        this.pBest = modules.get(0).getP();
        this.iBest = modules.get(0).getI();

        this.setpoint = setpoint;
        this.testDuration = testDuration;

        testers = new ArrayList<>();
        for(int i = 0; i < modules.size(); i++){
            testers.add(
                new Tester(modules.get(i))
            );
        }

        debug = new SmartDashString("AutoPID", "", true);
//        start();
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
            if(trial > 1) walk();
            start();
            trial++;
            return;
        }

        for(Tester t : testers){
            t.execute();
        }
    }
//
    //TODO: rename this method
    private void walk(){

        for(Tester tester : testers){
            double error = tester.getError();

            if(error < bestError && error != 0.0){
                bestError = error;
                pBest = tester.getModule().getP();
                iBest = tester.getModule().getI();

                debug.set("P: " + pBest + " I: " + iBest + " Error: " + error);
            }
        }

        for(Tester tester : testers){
            tester.getModule().setP(walkValue(pBest));
            tester.getModule().setI(walkValue(iBest));
        }

    }

    //TODO: rename this method
    private double walkValue(double value){
        return value * (1 + walkPercent * (2 * Math.random() - 1));
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
            t.getModule().setP(pBest);
            t.getModule().setI(iBest);
            t.getModule().savePI();
        }
        testers.forEach(Tester::end);
    }

    public class Tester{

        private AutoPIable module;

        private double startTime;
        private double error;
        private boolean running;

        public Tester(AutoPIable module) {
            this.module = module;
        }

        private void start(){
            startTime = Clock.getSeconds();
            error = 0;
            running = true;
        }

        private boolean hasEnded(){
            switch(module.getErrorMethod()){
                case DIFFERENCE:
                    return !running && Math.abs(module.tunerGet()) < 0.01;
                case ANGLE_DIFFERENCE:
                    return !running && Math.abs(SwerveChassis.getAngleDifference(0, module.tunerGet())) < 0.25;
            }
            return true;
        }

        private void execute() {
            if (!running){
                end();
                return;
            }

            if(getElapsed() > testDuration){
                end();
                return;
            }

            getError();

            module.tunerSet(setpoint);
        }

        private void end(){
            running = false;
            module.tunerSet(0.0);
        }

        private double getError(){
            if(!running){
                return error;
            }
            //If process is still running, add to the accumulative error.
            //Is integral between set and measured
            double diff = 0.0;
            double set = setpoint;
            double mes = module.tunerGet();
            double mod = 1;

            if(trial == 0){
                mod = 100;
            }

            switch(module.getErrorMethod()){
                case DIFFERENCE:
                    diff = set - mes;
                    if(diff < 0){
                        mod = 500;
                    }

                    break;
                case ANGLE_DIFFERENCE:
                    diff = SwerveChassis.getAngleDifference(set, mes);
                    if(diff < 0){
                        mod = 25;
                    }
                    break;
            }

            error += Math.abs(dt * diff) * mod;
            return error;
        }

        private double getElapsed(){
            return Clock.getSeconds() - startTime;
        }

        public AutoPIable getModule() {
            return module;
        }
    }

}
