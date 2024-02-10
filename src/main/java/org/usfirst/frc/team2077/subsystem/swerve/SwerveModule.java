package org.usfirst.frc.team2077.subsystem.swerve;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.DriveModuleIF;
import org.usfirst.frc.team2077.drivetrain.SwerveModuleIF;

public class
SwerveModule implements Subsystem, DriveModuleIF, SwerveModuleIF {

    public enum MotorPosition{

        FRONT_LEFT(2, 1, 5, 1.5),//Based on some of the numbers from ZTPHVN, TODO: check these
        BACK_LEFT(4, 3, 5, 1),
        BACK_RIGHT(6, 5, 5, 0.5),
        FRONT_RIGHT(8,7, 5, 0),
        ;

        public int drivingCANid;
        public int guidingCANid;
        public double maxSpeed;
        public double angleOffset;
        MotorPosition(int drivingCANid, int guidingCANid, double maxSpeed, double angleOffset){
            this.drivingCANid = drivingCANid;
            this.guidingCANid = guidingCANid;
            this.maxSpeed = maxSpeed;
            this.angleOffset = angleOffset * Math.PI;
        }
    }

    private final MotorPosition position;

    public boolean calibrating = false;

    private final SwerveDrivingMotor drivingMotor;
    private final SwerveGuidingMotor guidingMotor;

    public SwerveModule(MotorPosition position){
        this.position = position;

        drivingMotor = new SwerveDrivingMotor(position, this);
        guidingMotor = new SwerveGuidingMotor(position, this);

        this.register();
    }

    @Override
    public void periodic(){
        if(calibrating) return;

        drivingMotor.update();
        guidingMotor.update();
    }

    @Override
    public void setVelocity(double velocity) {
        drivingMotor.setVelocity(velocity);
    }

    @Override
    public void setAngle(double angle) {
        guidingMotor.setAngle(angle);
    }

    @Override
    public WheelPosition getWheelPosition() {
        return WheelPosition.valueOf(position.name());
    }

    @Override
    public double getVelocitySet() {
        return drivingMotor.getVelocitySet();
    }

    @Override
    public double getVelocityMeasured() {
        return drivingMotor.getVelocityMeasured();
    }

    @Override
    public double getAngle() {
        return guidingMotor.getAngle();
    }

    @Override
    public double getMaximumSpeed() {
        return position.maxSpeed;
    }

    public SwerveDrivingMotor getDrivingMotor(){
        return drivingMotor;
    }

    public SwerveGuidingMotor getGuidingMotor(){
        return guidingMotor;
    }

}
