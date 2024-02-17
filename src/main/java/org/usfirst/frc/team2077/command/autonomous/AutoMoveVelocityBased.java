package org.usfirst.frc.team2077.command.autonomous;


import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.VelocityDirection;
import org.usfirst.frc.team2077.common.drivetrain.AbstractChassis;
import org.usfirst.frc.team2077.common.math.Position;


import java.util.Map;

public class AutoMoveVelocityBased extends CommandBase {

    private static final double arbitraryAcceleration = 6.0;

    private AbstractChassis chassis;

    private Position from, to;
    private double remainingDistance;

    private double speed;
    private double minSpeed;

    private double forward;
    private double strafe;
    private double direction;

    public AutoMoveVelocityBased(double forward, double strafe){
        this.forward = forward;
        this.strafe = strafe;
    }

    private double lastTime;

    private double getDeltaTime(){
        double t = Clock.getSeconds();
        double dt = t - lastTime;
        lastTime = t;
        return dt;
    }
    @Override
    public void initialize() {
        lastTime = Clock.getSeconds();

        chassis = RobotHardware.getInstance().getChassis();

        from = chassis.getPosition();
        to = from.copy(); to.move(forward, strafe, 0);
        direction = Math.atan2(forward, strafe);

        remainingDistance = Math.hypot(forward, strafe);

        speed = (double) chassis.getMaximumVelocity().get(VelocityDirection.FORWARD) * 0.5;
        minSpeed = speed * 0.05;

        RobotHardware.getInstance().getChassis().setFieldOriented(false);
    }

    @Override
    public void execute() {
        Map<VelocityDirection, Double> currentVelocity = chassis.getVelocityMeasured();

        double dt = getDeltaTime();
        double measuredSpeed = Math.hypot(currentVelocity.get(VelocityDirection.FORWARD), currentVelocity.get(VelocityDirection.STRAFE));

        remainingDistance -= measuredSpeed;

        double stoppingDistance = Math.pow(measuredSpeed, 2) / arbitraryAcceleration;

        if(stoppingDistance > remainingDistance){
            speed -= arbitraryAcceleration;
            speed = Math.max(minSpeed, measuredSpeed);
        }

        chassis.setVelocityPercent(speed * Math.sin(direction) , speed * Math.cos(direction) );
    }

    @Override
    public void end(boolean interrupted) {
        RobotHardware.getInstance().getChassis().setFieldOriented(true);
        chassis.halt();
    }

    @Override public boolean isFinished(){
        return (remainingDistance <= 0);
    }

}
