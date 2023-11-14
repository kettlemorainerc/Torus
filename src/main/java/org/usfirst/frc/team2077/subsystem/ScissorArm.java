package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.common.subsystem.InputMap;

public class ScissorArm implements Subsystem {

    public enum Input{
        EXTEND;
    }

    private static final double PIVOT_MAX_SPEED = 0.5;
    private static final double EXTEND_MAX_SPEED = 0.5;

//    private static final double MAX_SCREW_RPM = ...;
    private static final double buffer = 1.2;

    private final CANSparkMax extensionMotor = new CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final TalonSRX pivotMotor = new TalonSRX(10);

    public ScissorArm(){
        this.register();
    }

    @Override public void periodic() {
        double extendPercent = -InputMap.getInput(Input.EXTEND); //These are backwards of how we think they should be
//        double pivotPercent = -InputMap.getInput("Pivot");

//        double appliedVoltage = extensionMotor.getBusVoltage();

        setExtensionMotor(extendPercent);
//        setPivotMotor(pivotPercent);
    }

    private void setExtensionMotor(double percent){
        percent *= EXTEND_MAX_SPEED;
        extensionMotor.set(percent);
    }

    private void setPivotMotor(double percent){
        percent *= PIVOT_MAX_SPEED;
        pivotMotor.set(TalonSRXControlMode.PercentOutput, percent);
    }



}
