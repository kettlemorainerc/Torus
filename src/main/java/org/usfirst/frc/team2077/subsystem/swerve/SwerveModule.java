package org.usfirst.frc.team2077.subsystem.swerve;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.DriveModuleIF;
import org.usfirst.frc.team2077.drivetrain.SwerveModuleIF;

public class
SwerveModule implements Subsystem, DriveModuleIF, SwerveModuleIF {

    public enum MotorPosition{

//        PID tuning finished
//                 ===FRONT_RIGHT_GUIDING_MOTOR=== 1.80013e-01, 3.85633e-05
//                 ===BACK_RIGHT_GUIDING_MOTOR===  2.63287e-01, 4.87870e-05
//                 ===BACK_LEFT_GUIDING_MOTOR=== 1.89910e-01, 3.22729e-05
//                 ===FRONT_LEFT_GUIDING_MOTOR=== 2.73380e-01, 6.20462e-05

        FRONT_LEFT (2, 1, 1.5, 2.14861e-02, 1.03070e-03, 2.73380e-01, 6.20462e-05),
        BACK_LEFT  (8, 7, 1,   4.36126e-02, 1.01244e-03, 1.89910e-01, 3.22729e-05),
        BACK_RIGHT (6, 5, 0.5, 4.11272e-02, 1.24478e-03, 2.63287e-01, 4.87870e-05),
        FRONT_RIGHT(4, 3, 0,   3.83776e-02, 8.66882e-04, 1.80013e-01, 3.85633e-05),
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

//        if(position == MotorPosition.FRONT_LEFT) notAllAtAngle = RobotHardware.getInstance().getChassis().getDriveModules().values().stream().allMatch(SwerveModule::isAtAngle);

        //if(!notAllAtAngle){
        drivingMotor.update();

        guidingMotor.update();

        //}

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

    /**
     * @Returns a value between 0 and 1 that represents how close the wheel is to its target angle.
     * This is used for throttle, for what percent the driving motor should be set to.
     * */
    public double dotToAngle(){
        return Math.cos(Math.abs(guidingMotor.distanceToTarget()));
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
