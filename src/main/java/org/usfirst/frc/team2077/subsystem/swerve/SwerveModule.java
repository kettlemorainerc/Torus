package org.usfirst.frc.team2077.subsystem.swerve;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.DriveModuleIF;
import org.usfirst.frc.team2077.drivetrain.SwerveModuleIF;

public class
SwerveModule implements Subsystem, DriveModuleIF, SwerveModuleIF {

    public enum MotorPosition{

        FRONT_LEFT (2, 1, 1.5, 0.02534497156739235, 5.266545340418816E-4, 0.21738624169716303,4.901154428286607E-5),
        BACK_LEFT  (8, 7, 1,   0.0271705724298954, 5.447675357572734E-4, 0.13649542924766456,4.274157968845806E-5),
        BACK_RIGHT (6, 5, 0.5, 0.028750400990247726, 5.698913591913879E-4, 0.1456683635071653, 3.8270833863588663E-5),
        FRONT_RIGHT(4, 3, 0,    0.2636384963989258, 9.083933605325E-9, 0.1559794460625504, 5.3351600006803205E-5),
        ;

        public final int drivingCANid, guidingCANid;
        public final double angleOffset;
        public final double drivingP, drivingI, guidingP, guidingI;
        MotorPosition(int drivingCANid, int guidingCANid, double angleOffset, double drivingP, double drivingI, double guidingP, double guidingI){
            this.drivingCANid = drivingCANid;
            this.guidingCANid = guidingCANid;
            this.angleOffset = angleOffset * Math.PI;
            this.drivingP = drivingP;
            this.drivingI = drivingI;
            this.guidingP = guidingP;
            this.guidingI = guidingI;
        }
    }

    private final MotorPosition position;

    public boolean calibrating = false;
    public boolean atAngle = false;

    public static boolean notAllAtAngle = false;

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

        if(position == MotorPosition.FRONT_LEFT) notAllAtAngle = RobotHardware.getInstance().getChassis().getDriveModules().values().stream().allMatch(SwerveModule::isAtAngle);

        //if(!notAllAtAngle){
        drivingMotor.update();
        //}

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

    public boolean isAtAngle(){
        return guidingMotor.atAngle();
    }

    @Override
    public double getMaximumSpeed() {
        return drivingMotor.getMaximumSpeed();
    }

    public SwerveDrivingMotor getDrivingMotor(){
        return drivingMotor;
    }

    public SwerveGuidingMotor getGuidingMotor(){
        return guidingMotor;
    }

}
