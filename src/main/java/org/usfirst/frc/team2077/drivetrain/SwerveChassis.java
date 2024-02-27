package org.usfirst.frc.team2077.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.AbstractChassis;
import org.usfirst.frc.team2077.common.drivetrain.DriveModuleIF;
import org.usfirst.frc.team2077.math.SwerveMath;
import org.usfirst.frc.team2077.math.SwerveTargetValues;
import org.usfirst.frc.team2077.subsystem.swerve.SwerveModule;

import java.util.Comparator;
import java.util.EnumMap;
import java.util.Map;

public class SwerveChassis extends AbstractChassis<SwerveModule> {

    public static final double wheelBaseLength = Units.inchesToMeters(36.5);//19.25);
    public static final double wheelBaseWidth = Units.inchesToMeters(36.5);//22.5);

    private final SwerveMath math;
    //ADIS16470_IMU is the class used in the provided swerve code //TODO: check gyro
//    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private final AHRS gyro = new AHRS();
    private double heading = 0.0;
    private boolean fieldOriented = true;

    private static EnumMap<WheelPosition, SwerveModule> buildDriveTrain() {
        EnumMap<WheelPosition, SwerveModule> map = new EnumMap<>(WheelPosition.class);

        for(SwerveModule.MotorPosition p : SwerveModule.MotorPosition.values()){
            map.put(WheelPosition.valueOf(p.name()), new SwerveModule(p));
        }

        return map;
    }

    public SwerveChassis() {
        super(buildDriveTrain());

//        gyro = new ADIS16470_IMU();

        math = new SwerveMath(wheelBaseLength, wheelBaseWidth);

        maximumSpeed = this.driveModules.values().stream().map(DriveModuleIF::getMaximumSpeed).min(Comparator.naturalOrder()).orElseThrow();

        //Dear David,
        //  I am sorry that I did not like you clever work around
        //  for finding the max rotation velocity, but it added a
        //  lot of clutter and is likely too confusion to anybody
        //  who did not watch you implement it directly.
        //Sincerely, Hank

        //TODO: confirm that this works
        double circumference = Math.PI * Math.hypot(wheelBaseLength, wheelBaseWidth);
        //The time it would take for a wheel traveling at maximum speed to travel the distance of the circumference
        double secondsPerRevolution = circumference / this.maximumSpeed;
        double radiansPerSecond = 2.0 * Math.PI / secondsPerRevolution;

        maximumRotation = radiansPerSecond;

        minimumSpeed = maximumSpeed * 0.1;
    }

    @Override protected void measureVelocity(){
        velocityMeasured = math.velocitiesForTargets(driveModules);
    }

    @Override protected void updateDriveModules() {
//        System.out.println(velocitySet.get(FORWARD));

        double gyroOffset = Math.toRadians(gyro.getAngle());

        Map<WheelPosition, SwerveTargetValues> wheelTargets;

        if(fieldOriented) {
            wheelTargets = math.targetsForVelocities(
                    velocitySet,
                    maximumSpeed,
                    maximumRotation,
                    gyroOffset
            );
        }else{
            wheelTargets = math.targetsForVelocities(
                    velocitySet,
                    maximumSpeed,
                    maximumRotation
            );
        }

        wheelTargets.forEach((key, value) -> {
            SwerveModule module = this.driveModules.get(key);

            double velocity = maximumSpeed * 0.65 * Math.abs(value.getMagnitude());

            if(velocity > maximumSpeed) velocity = maximumSpeed;
            if(velocity > 0.01) velocity = Math.max(velocity, minimumSpeed);

            module.setAngle(value.getAngle());
            module.setVelocity(velocity);
        });
    }

    public void resetGyro(){
        gyro.reset();
    }

    public void setFieldOriented(boolean v){
        fieldOriented = v;
    }

    public static double getAngleDifference(double from, double to) {
        double diff = from - to;
        if(Math.abs(diff) > Math.PI) diff -= 2 * Math.PI * Math.signum(diff);
        return diff;
    }

    public static double getAngleDifferenceDegrees(double from, double to) {
        double diff = from - to;
        if(Math.abs(diff) > 180) diff -= 360 * Math.signum(diff);
        return diff;
    }
}