package org.usfirst.frc.team2077.subsystem.swerve;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.DriveModuleIF;
import org.usfirst.frc.team2077.drivetrain.SwerveModuleIF;

public class
SwerveModule implements Subsystem, DriveModuleIF, SwerveModuleIF {

    public enum MotorPosition{

        FRONT_LEFT (2, 1, 1.5, /*P:*/ 0.02048513852059841, /*I:*/ 5.435076891444623E-4, 0.18089016030756128, 3.8191069037146064E-5),
        BACK_LEFT  (8, 7, 1,   /*P:*/ 0.030933115631341934, /*I:*/ 6.17226876784116E-4, 0.13372843696123757, 4.350619756154357E-5),
        BACK_RIGHT (6, 5, 0.5, /*P:*/ 0.022237218916416168, /*I:*/ 6.017343257553875E-4, 0.12722720716493266, 3.367660130168929E-5),
        FRONT_RIGHT(4, 3, 0,    /*P:*/ 0.03256119787693024, /*I:*/ 7.328314241021872E-4, 0.12639004706079296, 3.2892147470342086E-5),
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
