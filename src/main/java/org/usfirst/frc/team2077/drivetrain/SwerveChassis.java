package org.usfirst.frc.team2077.drivetrain;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import org.usfirst.frc.team2077.common.*;
import org.usfirst.frc.team2077.common.drivetrain.AbstractChassis;
import org.usfirst.frc.team2077.common.drivetrain.DriveModuleIF;
import org.usfirst.frc.team2077.math.SwerveMath;
import org.usfirst.frc.team2077.math.SwerveTargetValues;
import org.usfirst.frc.team2077.subsystem.SwerveModule;
import org.usfirst.frc.team2077.util.Constants.Frame;

import java.util.Comparator;
import java.util.EnumMap;
import java.util.Map;

import static org.usfirst.frc.team2077.common.VelocityDirection.*;

public class SwerveChassis extends AbstractChassis<SwerveModule> {

    private final SwerveMath math;
    //ADIS16470_IMU is the class used in the provided swerve code //TODO: check gyro
    private final /*AngleSensor*/ ADIS16470_IMU gyro;

    private static EnumMap<WheelPosition, SwerveModule> buildDriveTrain() {
        EnumMap<WheelPosition, SwerveModule> map = new EnumMap<>(WheelPosition.class);

        for(SwerveModule.MotorPosition p : SwerveModule.MotorPosition.values()){
            map.put(WheelPosition.valueOf(p.name()), new SwerveModule(p));
        }

        return map;
    }

    public SwerveChassis() {
        super(buildDriveTrain());

        gyro = new ADIS16470_IMU();

        math = new SwerveMath(Frame.kWheelBaseLength, Frame.kWheelBaseWidth);

        this.maximumSpeed = this.driveModules.values().stream().map(DriveModuleIF::getMaximumSpeed).min(Comparator.naturalOrder()).orElseThrow();

        // Hank, and anyone else. This should hopefully be more clear.

        // percent of maximum velocity
        double zeroPercent = 0, oneHundredPercent = 1;
        Map<VelocityDirection, Double> botVelocity = Map.of(FORWARD, zeroPercent, STRAFE, zeroPercent, ROTATION, oneHundredPercent);

        // Percents in, percents out
        Map<WheelPosition, SwerveTargetValues> wheelTargets = math.wheelStatesForBotVelocity(botVelocity);

        // "idealWheelStates" for purely rotation velocity
        Map<WheelPosition, SimpleSwerveWheelState> idealWheelStates = new EnumMap<>(WheelPosition.class);
        wheelTargets.forEach((k, v) -> {
            // degrees or radians or whatever
            double pureRotationIdealAngle = v.getAngle();
            // convert "% Max Velocity" -> "whatever unit maximum speed is in"
            double pureRotationIdealMagnitude = v.getMagnitude() * maximumSpeed;
            idealWheelStates.put(k, new SimpleSwerveWheelState(k, pureRotationIdealAngle, pureRotationIdealMagnitude));
        });

        // "calcluatedMaximumVelocity" for ideal rotational velocity wheel states
        Map<VelocityDirection, Double> calculatedMaximumVelocity = math.botVelocityForWheelStates(idealWheelStates);

        maximumRotation = calculatedMaximumVelocity.get(ROTATION);

        this.minimumSpeed = this.maximumSpeed * 0.1;
    }

    @Override protected void measureVelocity(){
        velocityMeasured = math.botVelocityForWheelStates(driveModules);
    }

    @Override protected void updateDriveModules() {
        Map<WheelPosition, SwerveTargetValues> wheelTargets = math.wheelStatesForBotVelocity(
            velocitySet,
            maximumSpeed,
            maximumRotation
        );

        wheelTargets.forEach((key, value) -> {
            SwerveModule module = this.driveModules.get(key);
            double velocity = Math.abs(value.getMagnitude());
            if(velocity > maximumSpeed) velocity = maximumSpeed;
            if(velocity > 0.01) velocity = Math.max(velocity, minimumSpeed);

            module.setAngle(value.getAngle());
            module.setVelocity(velocity);
        });

    }

    private static final class SimpleSwerveWheelState implements SwerveWheelState {
        private double angle, velocity;
        private final WheelPosition position;

        SimpleSwerveWheelState(WheelPosition position, double angle, double velocity) {
            this.position = position;
            this.angle = angle;
            this.velocity = velocity;
        }

        @Override public void setAngle(double angle) {this.angle = angle;}
        @Override public double getAngle() {return angle;}

        @Override public double getMaximumSpeed() {return Double.MAX_VALUE;}
        @Override public void setVelocity(double velocity) {this.velocity = velocity;}
        @Override public double getVelocitySet() {return velocity;}
        @Override public double getVelocityMeasured() {return velocity;}

        @Override public WheelPosition getWheelPosition() {return position;}
    }
}