package org.usfirst.frc.team2077.command.autonomous;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.drivetrain.AbstractChassis;


public class AutoRotate extends CommandBase {
    private final double DEGREES_TO_ENCODER_TICKS = 0.1875;
    private double lastTime;
    private AbstractChassis chassis;
    private RobotHardware hardware;
    private double[] prevEncoders;
    private double[] currentEncoders;
    private double targetAngle;
    private double angleSubtraction;
    private AHRS gyro = new AHRS();

    private double gyroAngle;
    private double initialAngle;
    public AutoRotate(double angle){
        targetAngle = angle;
        //targetAngle *= DEGREES_TO_ENCODER_TICKS;
    }

    @Override
    public void initialize(){
        lastTime = Clock.getSeconds();
        chassis = RobotHardware.getInstance().getChassis();
        hardware = RobotHardware.getInstance();

        initialAngle = gyro.getAngle();



        //prevEncoders = new double[]{hardware.getWheel(FRONT_LEFT).getDrivingEncoderPosition(), hardware.getWheel(FRONT_RIGHT).getDrivingEncoderPosition(), hardware.getWheel(BACK_RIGHT).getDrivingEncoderPosition(), hardware.getWheel(BACK_LEFT).getDrivingEncoderPosition() };



    }

    @Override
    public void execute(){

        /*//encoder based:
        currentEncoders = new double[]{hardware.getWheel(FRONT_LEFT).getDrivingEncoderPosition(), hardware.getWheel(FRONT_RIGHT).getDrivingEncoderPosition(), hardware.getWheel(BACK_RIGHT).getDrivingEncoderPosition(), hardware.getWheel(BACK_LEFT).getDrivingEncoderPosition()};
        angleSubtraction = Math.abs(prevEncoders[0] - currentEncoders[0]) + Math.abs(prevEncoders[1] - currentEncoders[1]) + Math.abs(prevEncoders[2] - currentEncoders[2]) + Math.abs(prevEncoders[3] - currentEncoders[3]);
        angleSubtraction *= 0.25;
        */

        // encoder based:
        gyroAngle = initialAngle - gyro.getAngle();
//        System.out.println(gyroAngle);





        if (isFinished()) {this.end(false);}
        chassis.setVelocityPercent(0,0,0.3);



//        System.out.println("Back left:");
//            System.out.println(hardware.getWheel(BACK_LEFT).getDrivingEncoderPosition());
//        System.out.println("Front left:");
//            System.out.println(hardware.getWheel(FRONT_LEFT).getDrivingEncoderPosition());
//        System.out.println("Back right:");
//            System.out.println(hardware.getWheel(BACK_RIGHT).getDrivingEncoderPosition());
//        System.out.println("Front right:");
//            System.out.println(hardware.getWheel(FRONT_RIGHT).getDrivingEncoderPosition());


    }




    @Override
    public void end(boolean interrupted) {
        chassis.halt();
    }



    @Override public boolean isFinished(){
        return(gyroAngle >= targetAngle);

        //encoder based:
        //return (angleSubtraction >= targetAngle);
    }
}
