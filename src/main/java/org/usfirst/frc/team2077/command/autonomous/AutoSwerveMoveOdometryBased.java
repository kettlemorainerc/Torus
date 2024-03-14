package org.usfirst.frc.team2077.command.autonomous;


import edu.wpi.first.wpilibj2.command.Command;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.math.Position;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.util.SmartDashRobotPreference;

import static org.usfirst.frc.team2077.common.VelocityDirection.*;

public class AutoSwerveMoveOdometryBased extends Command {

    //TODO: find out what these values should be
    private static final SmartDashRobotPreference cardinal_p = new SmartDashRobotPreference("Auto Cardinal P", 0.1);
    private static final SmartDashRobotPreference rotation_p = new SmartDashRobotPreference("Auto Cardinal I", 0.1);

    private static final double cardinalGoodEnoughThreshold = 0.5; //Meters
    private static final double rotationGoodEnoughThreshold = Math.PI / 8; //Radians

    protected SwerveChassis chassis;
    protected Position target;

    private boolean finished;

    public AutoSwerveMoveOdometryBased(double forward, double strafe, double rotation){
        chassis = RobotHardware.getInstance().getChassis();
        target = chassis.getPosition();
        target.move(forward, strafe, rotation);//IMO, move should return the position itself
    }

    public AutoSwerveMoveOdometryBased(double forward, double strafe){
        this(forward, strafe, 0.0);
    }

    public AutoSwerveMoveOdometryBased(double rotation){
        this(0.0, 0.0, rotation);
    }

    @Override
    public void initialize() {
        finished = false;
        chassis.setFieldOriented(false);
    }

    @Override
    public void execute() {
        //This should be capable of driving to a cardinal and rotational target simultaneously
        Position current = chassis.getPosition();

        double deltaForward = target.get(FORWARD) - current.get(FORWARD);
        double deltaStrafe = target.get(STRAFE) - current.get(STRAFE);

        double deltaRotation = SwerveChassis.getAngleDifference(target.get(ROTATION), current.get(ROTATION));

        if(
            Math.abs(deltaForward) < cardinalGoodEnoughThreshold &&
            Math.abs(deltaStrafe) < cardinalGoodEnoughThreshold &&
            Math.abs(deltaRotation) < rotationGoodEnoughThreshold
            //It might also be beneficial to do a check to make sure the robot's velocity is close to zero
        ){
            finished = true;
//            return;
        }

        double forwardVelocity = deltaForward * cardinal_p.get();
        double strafeVelocity = deltaStrafe * cardinal_p.get();

        double targetSpeed = Math.hypot(forwardVelocity, strafeVelocity);
        double maxSpeed = chassis.getMaximumVelocity().get(FORWARD);

        //Keeps velocities passed into chassis under or at max speed;
        if(targetSpeed > maxSpeed){
            forwardVelocity = forwardVelocity * maxSpeed / targetSpeed;
            strafeVelocity = strafeVelocity * maxSpeed / targetSpeed;
        }

        double rotationVelocity = deltaRotation * rotation_p.get();

        double maxRotation = chassis.getMaximumVelocity().get(ROTATION);
        if(Math.abs(rotationVelocity) > maxRotation){
            rotationVelocity = maxRotation * Math.signum(rotationVelocity);
        }

        chassis.setVelocity(forwardVelocity, strafeVelocity, rotationVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.halt(); //Just a precaution
        chassis.setFieldOriented(true);
    }

    @Override public boolean isFinished(){
        return finished;
    }

}
