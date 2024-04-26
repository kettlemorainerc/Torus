package org.usfirst.frc.team2077.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.AbstractChassis;
import org.usfirst.frc.team2077.common.drivetrain.DriveModuleIF;
import org.usfirst.frc.team2077.common.math.Vector;
import org.usfirst.frc.team2077.math.SwerveMath;
import org.usfirst.frc.team2077.math.SwerveWheelTarget;
import org.usfirst.frc.team2077.subsystem.swerve.SwerveModule;

import java.util.Comparator;
import java.util.EnumMap;
import java.util.Map;

public class SwerveChassis extends AbstractChassis<SwerveModule> {

    public static final double wheelBaseLength = Units.inchesToMeters(29.5);//19.25);
    public static final double wheelBaseWidth = Units.inchesToMeters(29.5);//22.5);

    public enum DriveMode{
        BRAKE, COAST, ANGLE_REQ;
    }

    public DriveMode mode = DriveMode.COAST;

    private final SwerveMath math;
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

        maximumSpeed = this.driveModules.values().stream().map(DriveModuleIF::getMaximumSpeed).min(Comparator.naturalOrder()).orElseThrow();

        //Dear David,
        //  I forgot what I was complaining about
        //Sincerely, Hank

        double circumference = Math.PI * Math.hypot(wheelBaseLength, wheelBaseWidth);
        double secondsPerRevolution = circumference / this.maximumSpeed;
        double radiansPerSecond = 2.0 * Math.PI / secondsPerRevolution;

        maximumRotation = radiansPerSecond;

        minimumSpeed = maximumSpeed * 0.1;

        math = new SwerveMath(wheelBaseLength, wheelBaseWidth, maximumSpeed, maximumRotation);
    }

    @Override protected void measureVelocity(){
        velocityMeasured = math.velocitiesForTargets(driveModules);
    }

    @Override protected void updateDriveModules() {
//        System.out.println(velocitySet.get(FORWARD));

        Vector target = velocitySet.copy();
        if(fieldOriented){
            double gyroOffset = Math.toRadians(gyro.getAngle());
            target.rotate(gyroOffset);
        }

        Map<WheelPosition, SwerveWheelTarget> wheelTargets = math.getWheelTargets(velocitySet, maximumSpeed, maximumRotation);

        wheelTargets.forEach((key, value) -> {
            SwerveModule module = this.driveModules.get(key);

            double velocity = maximumSpeed * 0.65 * Math.abs(value.getMagnitude());

            if(velocity > maximumSpeed) velocity = maximumSpeed;
            if(velocity > 0.01) velocity = Math.max(velocity, minimumSpeed);

            module.setVelocity(velocity);
            module.setAngle(value.getAngle());
        });
    }

    public void resetGyro(){
        gyro.reset();
    }

    public void setFieldOriented(boolean v){
        fieldOriented = v;
    }

    public static double getAngleDifference(double to, double from) {
        double diff = from - to;
        if(Math.abs(diff) > Math.PI) diff -= 2 * Math.PI * Math.signum(diff);
        return diff;
    }

    public static double getAngleDifferenceDegrees(double to, double from) {
        double diff = from - to;
        if(Math.abs(diff) > 180) diff -= 360 * Math.signum(diff);
        return diff;
    }
}