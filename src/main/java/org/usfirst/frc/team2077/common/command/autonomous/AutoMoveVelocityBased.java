package org.usfirst.frc.team2077.common.command.autonomous;


import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.VelocityDirection;
import org.usfirst.frc.team2077.common.drivetrain.AbstractChassis;
import org.usfirst.frc.team2077.common.math.Position;


import java.util.Map;

public class AutoMoveVelocityBased extends CommandBase {

    private final double SPEED_LIMITER = 0.6;
    private final double CONVERSION_VALUE = 0.25;

    private AbstractChassis chassis;

    private Position from, to;
    private double remainingDistance;

    private double forward;
    private double strafe;
    private double direction;
    private double forwardMultiplier;
    private double strafeMultiplier;

    public AutoMoveVelocityBased(double forward, double strafe){
        this.forward = forward * CONVERSION_VALUE;
        this.strafe = strafe * CONVERSION_VALUE;
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

        forwardMultiplier = Math.sin(direction);
        strafeMultiplier = Math.cos(direction);

        RobotHardware.getInstance().getChassis().setFieldOriented(false);
    }

    @Override
    public void execute() {
        Map<VelocityDirection, Double> currentVelocity = chassis.getVelocityMeasured();

        double dt = getDeltaTime();
        double forwardVelocity = currentVelocity.get(VelocityDirection.FORWARD);
        double strafeVelocity = currentVelocity.get(VelocityDirection.STRAFE);

        remainingDistance -= Math.hypot(forwardVelocity * dt, strafeVelocity * dt);

        if(isFinished()){this.end(false);}
        chassis.setVelocityPercent(forwardMultiplier * SPEED_LIMITER , strafeMultiplier * SPEED_LIMITER );
    }

    @Override
    public void end(boolean interrupted) {
        chassis.halt();
    }

    @Override public boolean isFinished(){
        return(remainingDistance <= 0);
    }

}
