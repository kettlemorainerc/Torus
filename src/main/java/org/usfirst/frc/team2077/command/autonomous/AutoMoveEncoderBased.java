package org.usfirst.frc.team2077.command.autonomous;


import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.AbstractChassis;
import org.usfirst.frc.team2077.common.math.Position;

public class AutoMoveEncoderBased extends CommandBase {

    private final double SPEED_LIMITER = 0.6;

    private AbstractChassis chassis;

    private Position from, to;
    private double remainingDistance;

    private double forward;
    private double strafe;
    private double direction;
    private double forwardMultiplier;
    private double strafeMultiplier;

    private double lastTime;

    private double startEncoders;
    private double currentEncoders;

    public AutoMoveEncoderBased(double forward, double strafe){
        this.forward = forward;
        this.strafe = strafe;
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


        startEncoders = RobotHardware.getInstance().getWheel(WheelPosition.FRONT_RIGHT).getDrivingEncoderPosition() +
                                      RobotHardware.getInstance().getWheel(WheelPosition.BACK_RIGHT).getDrivingEncoderPosition() +
                                      RobotHardware.getInstance().getWheel(WheelPosition.BACK_LEFT).getDrivingEncoderPosition() +
                                      RobotHardware.getInstance().getWheel(WheelPosition.FRONT_LEFT).getDrivingEncoderPosition();
        startEncoders *= 0.25;

        RobotHardware.getInstance().getChassis().setFieldOriented(false);
    }

    @Override
    public void execute() {


        currentEncoders = RobotHardware.getInstance().getWheel(WheelPosition.FRONT_RIGHT).getDrivingEncoderPosition() +
                RobotHardware.getInstance().getWheel(WheelPosition.BACK_RIGHT).getDrivingEncoderPosition() +
                RobotHardware.getInstance().getWheel(WheelPosition.BACK_LEFT).getDrivingEncoderPosition() +
                RobotHardware.getInstance().getWheel(WheelPosition.FRONT_LEFT).getDrivingEncoderPosition();
        currentEncoders *= 0.25;
        remainingDistance = startEncoders-currentEncoders * -1;

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
