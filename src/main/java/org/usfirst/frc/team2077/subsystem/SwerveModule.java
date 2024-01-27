package org.usfirst.frc.team2077.subsystem;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.RobotHardware;
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

    private static final double driveGearReduction = (45d * 22d) / (15d * 13d/*This is the variable gear*/);
    private static final int drivingMotorCurrentLimit = 50; // amps
    private static final int guidingMotorCurrentLimit = 20; // amps

    private static final double angleDeadZone = 0.25;

    public static final double wheelRadius = Units.inchesToMeters(2);
    public static final double wheelDiameter = wheelRadius * 2.0;
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static boolean notAllAtAngle = false;

    private final MotorPosition position;

    private final CANSparkMax guidingMotor;
    private final AbsoluteEncoder guidingEncoder;
    private final PIDController guidingPID;
    private final SparkMaxPIDController guidingCANPID;

    private final CANSparkMax drivingMotor;
    private final RelativeEncoder drivingEncoder;
    private final SparkMaxPIDController drivingPID;

    private boolean calibrating = false;
    private double angleOffset = 0.0;

    private double velocitySet;
    private double angleSet = 0;

    private boolean flipMagnitude = false;

    //P: 0.12839818, I: 0.00006161

    public SwerveModule(MotorPosition position){
        this.position = position;
        angleOffset = position.angleOffset;

        //"Factory reset, so we get the SPARKS MAX to a known state before configuring"
        //"them. This is useful in case a SPARK MAX is swapped out."
//        drivingMotor.restoreFactoryDefaults();
//        guidingMotor.restoreFactoryDefaults();

        guidingMotor = new CANSparkMax(position.guidingCANid, MotorType.kBrushless);
        guidingMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        guidingMotor.setSmartCurrentLimit(guidingMotorCurrentLimit);

        guidingEncoder = guidingMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        guidingEncoder.setPositionConversionFactor(2.0 * Math.PI);
        guidingEncoder.setInverted(false);

        guidingCANPID = guidingMotor.getPIDController();
        guidingPID = new PIDController(guidingCANPID.getP(), guidingCANPID.getI(), 0.0);

//        guidingPID.setFeedbackDevice(guidingEncoder);
//        guidingPID.setPositionPIDWrappingEnabled(true);
//        guidingPID.setPositionPIDWrappingMinInput(0);
//        guidingPID.setPositionPIDWrappingMaxInput(2.0 * Math.PI);
//        guidingPID.setOutputRange(-1, 1);

        // "Save the SPARK MAX configurations. If a SPARK MAX browns out during"
        // "operation, it will maintain the above configurations."
        guidingMotor.burnFlash();

        //Setting up the driving motor
        drivingMotor = new CANSparkMax(position.drivingCANid, MotorType.kBrushless);
        drivingMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        drivingMotor.setSmartCurrentLimit(drivingMotorCurrentLimit);

        drivingMotor.setClosedLoopRampRate(1.0);

        drivingEncoder = drivingMotor.getEncoder();
        drivingEncoder.setVelocityConversionFactor(wheelCircumference / driveGearReduction / 60.0);

        drivingPID = drivingMotor.getPIDController();

        drivingMotor.burnFlash();
//        guidingMotor.burnFlash();

//        drivingPID.setP(0.08728395);
//        drivingPID.setI(0.00107429);


        this.register();
    }

    @Override
    public void periodic(){
        if(calibrating) return;

//        System.out.printf(
//                "P: %.8f, I: %.8f", guidingCANPID.getP(), guidingCANPID.getI()
//        );
//        System.out.println(velocitySet);

        guidingPeriodic();
        drivingPeriodic();
    }

    private void drivingPeriodic(){
//        if(!calibrating && getWheelPosition().ordinal() == 0) notAllAtAngle = RobotHardware.getInstance().getChassis().getDriveModules().values().stream().map(SwerveModule::isAtAngle).anyMatch(e -> !e);

//        if(!calibrating && notAllAtAngle && Math.abs(velocitySet) > 0.1) return;

        drivingPID.setReference(velocitySet * (flipMagnitude? -1 : 1), CANSparkMax.ControlType.kVelocity);
    }

    private void guidingPeriodic(){
        if(Math.abs(velocitySet) < 0.01 && !calibrating) {
            guidingMotor.set(0.0);
            return;
        }

        double angleDiff = getAngleDifference(angleSet, getAngle());
        double p = guidingPID.calculate(Math.abs(angleDiff), 0.0) * Math.signum(angleDiff);
        guidingMotor.set(p);
    }

    //Only used for PID
    public void calibrationSetVelocity(double velocity){
        calibrating = true;

        setVelocity(velocity);
        if(Math.abs(velocity) < 0.01){
            drivingMotor.set(0);
        }else{
            flipMagnitude = false;
            drivingPeriodic();
        }
    }

    @Override
    public void setVelocity(double velocity) {
        velocitySet = velocity;
    }

    //Only used for PID
    public void calibrationSetAngle(double angle){
        calibrating = true;

        if(angle == 0.0){
            guidingMotor.set(0.0);
            angleOffset -= getAngle();
            return;
        }

        angle %= 2.0 * Math.PI;
        if(angle < 0) angle += 2.0 * Math.PI;
        angleSet = angle;

        guidingPID.setP( guidingCANPID.getP() );
        guidingPID.setI( guidingCANPID.getI() );

        guidingPeriodic();
    }

    @Override
    public void setAngle(double angle) {
        double currentAngle = getAngle();
        double angleDifference = getAngleDifference(currentAngle, angle);

        if(
            Math.abs(velocitySet) > 0.1
        ){
            if(flipMagnitude){
                angle -= Math.PI;
            }
        }else if(Math.abs(angleDifference) > 0.5 * Math.PI) {
            angle -= Math.PI;
            flipMagnitude = true;
        }else{
            flipMagnitude = false;
        }

        angle %= 2.0 * Math.PI;
        if(angle < 0) angle += 2.0 * Math.PI;
        angleSet = angle;
    }

    public static double getAngleDifference(double from, double to) {
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
//        System.out.printf("Set%.2f drivingEncoder.getVelocity());
        return drivingEncoder.getVelocity();
    }

    @Override
    public double getAngle() {
        double angle = guidingEncoder.getPosition() + angleOffset;
        angle %= 2.0 * Math.PI;
        if(angle < 0) angle += 2.0 * Math.PI;
        return angle;
    }

    private boolean isAtAngle(){
        return Math.abs(getAngleDifference(angleSet, getAngle())) <= angleDeadZone;
    }

    @Override
    public double getMaximumSpeed() {
        return position.maxSpeed;
    }

    public SparkMaxPIDController getDrivingPID(){
        return drivingPID;
    }

    public SparkMaxPIDController getGuidingPID(){
        return guidingCANPID;
    }

    public void savePID(){
        REVLibError p;
        do{
            p = guidingMotor.burnFlash();
        }while(p != REVLibError.kOk);
        do{
            p = drivingMotor.burnFlash();
        }while(p != REVLibError.kOk);

        System.out.println("Saved?!");

//        System.out.printf(
//            "P: %.2f, I: %.2f", guidingCANPID.getP(), guidingCANPID.getI()
//        );
    }

}
