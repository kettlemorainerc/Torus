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

        RobotHardware.getInstance().getChassis().setFieldOriented(false);
    }

    @Override
    public void execute() {
        Map<VelocityDirection, Double> currentVelocity = chassis.getVelocityMeasured();

        double dt = getDeltaTime();
        double forwardVelocity = currentVelocity.get(VelocityDirection.FORWARD);
        double strafeVelocity = currentVelocity.get(VelocityDirection.STRAFE);
        double velocity = Math.hypot(forwardVelocity, strafeVelocity);

        remainingDistance -= Math.hypot(forwardVelocity * dt, strafeVelocity * dt);

        double stoppingDistance = Math.pow(velocity, 2) / arbitraryAcceleration;

        if(stoppingDistance > remainingDistance){

        }

        chassis.setVelocityPercent(forwardMultiplier * SPEED_LIMITER , strafeMultiplier * SPEED_LIMITER );
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
