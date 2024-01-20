package org.usfirst.frc.team2077.command;

import com.revrobotics.SparkMaxPIDController;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.command.SelfDefinedCommand;
import org.usfirst.frc.team2077.common.drivetrain.AbstractChassis;
import org.usfirst.frc.team2077.subsystem.SwerveModule;
import org.usfirst.frc.team2077.util.SmartDashNumber;
import org.usfirst.frc.team2077.util.SmartDashString;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

public class PIDAutoTune extends SelfDefinedCommand {

    private static double pGuess = 0.1396;
    private static double iGuess = 0.0011;

    private static double testingVelocity = 1; //m/s
    private static double testingTime = 4;//s
    private static double maxError = 15;//TODO: determine good error for this

    private static double walkingPercent = 0.33;

    private AbstractChassis chassis;
    private Map<WheelPosition, PIDTester> testers;

    private double pBest = 0;
    private double iBest = 0;

    private double pEntropy = 0;
    private double iEntropy = 0;

    private final double entropyDecay = 0.25;

    private SmartDashNumber error;
    private SmartDashNumber best;
    private SmartDashString bestPID;

    private WheelPosition mostBest = WheelPosition.BACK_LEFT;

    private double dt = 0;
    private double pt = 0;

    public PIDAutoTune(){
        chassis = RobotHardware.getInstance().getChassis();
        Map<WheelPosition, SwerveModule> driveModules = chassis.getDriveModules();
        testers = new EnumMap<>(WheelPosition.class);

        for(SwerveModule module : driveModules.values()){
            testers.put(module.getWheelPosition(),
                new PIDTester(module, module.getDrivingPID())
            );
        }

        error = new SmartDashNumber("PID Error", 0.0, true);
        best = new SmartDashNumber("Best Motor", 0.0, true);
        bestPID = new SmartDashString("Best PID", "", true);
        System.out.println("Init");

       pt = Clock.getSeconds();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void initialize() {
        startTest();
    }

    @Override
    public void execute() {
        if(!inTesting()){
            walk();
            System.out.println("----------startTest------");
            startTest();
            return;
        }

        double ct = Clock.getSeconds();
        dt = ct - pt;
        pt = ct;

        testers.values().forEach(PIDTester::execute);
    }

    private void walk(){
        ArrayList<PIDTester> testersArray = new ArrayList<>(testers.values());
        double minError = Double.MAX_VALUE;
        double pBest = 0, iBest = 0;
        for(PIDTester tester : testersArray){
            double error = tester.getError();
            if(error < minError){
                minError = error;
                pBest = tester.getPID().getP();
                iBest = tester.getPID().getI();
                mostBest = tester.module.getWheelPosition();

                this.error.set(error);

//                System.out.println("P: " + pBest + ", I: " + iBest);
                this.bestPID.set("P: " + pBest + ", I: " + iBest);
            }
        }

        if(this.pBest != 0) pEntropy = pBest - this.pBest;
        if(this.iBest != 0) iEntropy = iBest - this.iBest;

        this.pBest = pBest;
        this.iBest = iBest;

        for(PIDTester tester : testersArray){
            if(tester.module.getWheelPosition() != mostBest){
                tester.getPID().setP(
                    pBest * (1 + (2 * Math.random() - 0.5) * walkingPercent) + pEntropy * entropyDecay
                );
                tester.getPID().setI(
                    iBest * (1 + (2 * Math.random() - 0.5) * walkingPercent) + iEntropy * entropyDecay
                );
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        testers.values().forEach(PIDTester::end);
    }

    private boolean inTesting(){
        AtomicBoolean testing = new AtomicBoolean(false);
        testers.values().forEach(e -> {
            if(!e.hasEnded()) testing.set(true);
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
        private double accumError = 0;

        private boolean running = false;

        public PIDTester(SwerveModule module, SparkMaxPIDController pid){
            this.module = module;
            this.pid = pid;

            randomisePID();
        }

        private void randomisePID(){
            pid.setP( Math.random() * pGuess);
            pid.setI( Math.random() * iGuess);
        }

        private void start(){
            running = true;
            accumError = 0;
            startTime = Clock.getSeconds();
            pid.setIAccum(0);
        }

        private void execute(){
            if(!running) return;

            if(getElapsed() > testingTime || getError() > maxError) {
                end();
                return;
            }

            module.setVelocity(testingVelocity);
            getError();
//            System.out.printf("P %.2f I %.2f", pid.getP(), pid.getI());

            if(mostBest == module.getWheelPosition()){
                best.set(module.getVelocityMeasured());
            }
        }

        private void end(){
            testError = accumError;
            running = false;
            module.setVelocity(0);
        }

        public boolean hasEnded(){
            return !running && Math.abs(module.getVelocityMeasured()) < 0.1;
        }

        private double getError(){
            if(Math.abs(module.getVelocitySet()) < 0.01){
                return accumError;
            }
            double currentError = dt * (module.getVelocitySet() - module.getVelocityMeasured());
            accumError += Math.abs(currentError);
            return accumError;
        }

        private double getTestError(){
            return testError;
        }

        private double getElapsed(){
            return Clock.getSeconds() - startTime;
        }

        public SparkMaxPIDController getPID() {
            return pid;
        }

    }
}
