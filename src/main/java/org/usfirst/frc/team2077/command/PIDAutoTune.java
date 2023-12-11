package org.usfirst.frc.team2077.command;

import com.revrobotics.SparkMaxPIDController;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.common.drivetrain.AbstractChassis;
import org.usfirst.frc.team2077.subsystem.SwerveModule;
import org.usfirst.frc.team2077.util.SmartDashNumber;

import java.util.ArrayList;
import java.util.Collection;
import java.util.EnumMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Collectors;

public class PIDAutoTune extends RepeatedCommand {

    private static double pMaxRange = 1.0;
    private static double iMaxRange = 0.5;

    private static double testingVelocity = 5; //m/s
    private static double testingTime = 3;//s
    private static double maxError = 1;//TODO: figure out what the hell this should be

    private static double walkingPercent = 0.2;

    private AbstractChassis chassis;
    private Map<WheelPosition, PIDTester> testers;

//    private SmartDashNumber error;

    public PIDAutoTune(){
        chassis = RobotHardware.getInstance().getChassis();
        Map<WheelPosition, SwerveModule> driveModules = chassis.getDriveModules();
        testers = new EnumMap<>(WheelPosition.class);

        for(SwerveModule module : driveModules.values()){
            testers.put(module.getWheelPosition(),
                new PIDTester(module, module.getDrivingPID())
            );
        }

//        error = new SmartDashNumber("PID Error", 0.0, true);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(!inTesting()){
            PSO();
            startTest();
            return;
        }

        testers.values().forEach(PIDTester::execute);
    }

    //TODO: Rename this function dumbass
    private void PSO(){
        ArrayList<PIDTester> tests = new ArrayList<>(testers.values());
        double minError = Double.MAX_VALUE;
        double p_target = 0, i_target = 0;
        for(PIDTester test : tests){
            double error = test.getTestError();
            if(error < minError){
                minError = error;
                p_target = test.getPID().getP();
                i_target = test.getPID().getI();
            }
        }

        System.out.printf("Min Error %.2f at p: %.4f i: %.4f%n", minError, p_target, i_target);
//        error.set(minError);

        //TODO: improve walking function, but test this as a baseline
        for(PIDTester test : tests){
            double p = test.getPID().getP();
            double i = test.getPID().getI();

            double p_diff = p_target - p;
            double i_diff = i_target - i;

            p += p_diff * walkingPercent;
            i += i_diff * walkingPercent;

            test.getPID().setP(p);
            test.getPID().setI(i);
        }

    }

    @Override
    public void end(boolean interrupted) {
    }

    private boolean inTesting(){
        AtomicBoolean testing = new AtomicBoolean(false);
        testers.values().forEach(e -> {
            if(e.hasEnded()) testing.set(true);
        });
        return testing.get();
    }

    private void startTest(){
        testers.values().forEach(PIDTester::start);
    }

    private class PIDTester{

        private SwerveModule module;
        private SparkMaxPIDController pid;

        private double startTime = 0;
        private double testError = 0;

        private boolean running = false;

        public PIDTester(SwerveModule module, SparkMaxPIDController pid){
            this.module = module;
            this.pid = pid;

            randomisePID();
        }

        private void randomisePID(){
            pid.setP( Math.random() * pMaxRange );
            pid.setI( Math.random() * iMaxRange );
        }

        private void start(){
            running = true;
            testError = 0;
            startTime = Clock.getSeconds();
            pid.setIAccum(0);
        }

        private void execute(){
            if(!running) return;

            if(getElapsed() > testingTime || getError() > maxError) {
                running = false;
                module.setVelocity(0);
                testError = getError();
                return;
            }

            module.setVelocity(testingVelocity);
        }

        private double getTestError(){
            return testError;
        }

        public boolean hasEnded(){
            return !running && Math.abs(module.getVelocityMeasured()) < 0.1;
        }

        private double getError(){
            return pid.getIAccum();
        }

        private double getElapsed(){
            return Clock.getSeconds() - startTime;
        }

        public SparkMaxPIDController getPID() {
            return pid;
        }

    }
}
