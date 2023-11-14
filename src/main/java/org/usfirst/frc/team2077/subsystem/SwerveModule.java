package org.usfirst.frc.team2077.subsystem;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.DriveModuleIF;
import org.usfirst.frc.team2077.drivetrain.SwerveModuleIF;
import org.usfirst.frc.team2077.util.Constants.Drive;

public class SwerveModule implements Subsystem, DriveModuleIF, SwerveModuleIF {

    public enum MotorPosition{
        FRONT_LEFT(0, 1, -500),
        FRONT_RIGHT(2,3, -500),
        BACK_LEFT(4, 5, -500),
        BACK_RIGHT(6, 7, -500);

        public int drivingCANid;
        public int guidingCANid;
        public double maxSpeed;
        MotorPosition(int drivingCANid, int guidingCANid, double maxSpeed){
            this.drivingCANid = drivingCANid;
            this.guidingCANid = guidingCANid;
            this.maxSpeed = maxSpeed;
        }
    }

    private final MotorPosition position;

    private final CANSparkMax guidingMotor;
    private final AbsoluteEncoder guidingEncoder;
    private final SparkMaxPIDController guidingPID;

    private final CANSparkMax drivingMotor;
    private final RelativeEncoder drivingEncoder;
    private final SparkMaxPIDController drivingPID;

    private double velocitySet;
    private double angleSet;

    private boolean flipMagnitude;

    public SwerveModule(MotorPosition position){
        this.position = position;
        //"Factory reset, so we get the SPARKS MAX to a known state before configuring"
        //"them. This is useful in case a SPARK MAX is swapped out."
//        drivingMotor.restoreFactoryDefaults();
//        guidingMotor.restoreFactoryDefaults();

        //Setting up the guiding motor
        guidingMotor = new CANSparkMax(position.drivingCANid, MotorType.kBrushless);
        guidingMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        guidingMotor.setSmartCurrentLimit(Drive.kGuidingMotorCurrentLimit);

        guidingEncoder = guidingMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        guidingEncoder.setPositionConversionFactor(2d * Math.PI);
        guidingEncoder.setInverted(true);

        guidingPID = guidingMotor.getPIDController();
        guidingPID.setFeedbackDevice(guidingEncoder);
        guidingPID.setPositionPIDWrappingEnabled(true);
        guidingPID.setPositionPIDWrappingMinInput(0);
        guidingPID.setPositionPIDWrappingMinInput(2d * Math.PI);
        guidingPID.setOutputRange(-1, 1);

        // "Save the SPARK MAX configurations. If a SPARK MAX browns out during"
        // "operation, it will maintain the above configurations."
        guidingMotor.burnFlash();

        //Setting up the driving motor
        drivingMotor = new CANSparkMax(position.drivingCANid, MotorType.kBrushless);
        drivingMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        drivingMotor.setSmartCurrentLimit(Drive.kDrivingMotorCurrentLimit);

        drivingEncoder = drivingMotor.getEncoder();
        drivingEncoder.setVelocityConversionFactor(Drive.kWheelCircumference / Drive.kDriveGearReduction / 60.0);

        drivingPID = drivingMotor.getPIDController();
        drivingPID.setFeedbackDevice(drivingEncoder);
        drivingPID.setOutputRange(-1, 1);

        drivingMotor.burnFlash();
    }

    @Override
    public void periodic(){
        guidingPID.setReference(angleSet, CANSparkMax.ControlType.kPosition);
        drivingPID.setReference(velocitySet * (flipMagnitude? -1 : 1), CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setVelocity(double velocity) {
        velocitySet = velocity;
    }

    @Override
    public void setAngle(double angle) {
        double currentWheelAngle = getAngle();
        double angleDifference = getAngleDifference(currentWheelAngle, angle);

        flipMagnitude = false;
        if(Math.abs(angleDifference) > 0.5 * Math.PI) {
            angle -= Math.PI;
            flipMagnitude = true;
        }

        angle %= 2 * Math.PI;
        angleSet = angle;
    }

    private double getAngleDifference(double from, double to) {
        double diff = from - to;
        if(Math.abs(diff) > Math.PI) diff -= 2 * Math.PI * Math.signum(diff);
        return diff;
    }

    @Override
    public WheelPosition getWheelPosition() {
        return WheelPosition.valueOf(position.name());
    }

    @Override
    public double getVelocitySet() {
        return velocitySet;
    }

    @Override
    public double getVelocityMeasured() {
        return drivingEncoder.getVelocity();
    }

    @Override
    public double getAngle() {
        return guidingEncoder.getPosition();
    }

    @Override
    public double getMaximumSpeed() {
        return position.maxSpeed;
    }

}
