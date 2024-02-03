package org.usfirst.frc.team2077.command;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.util.SmartDashNumber;

public class RunLauncher extends RepeatedCommand {

    private TalonSRX motor;
    private SmartDashNumber dash;
    private int d = 0;

    public RunLauncher(int deviceNumber, String key, int d){
        motor = new TalonSRX(deviceNumber);
        dash = new SmartDashNumber(key, 0.0, true);
        motor.setNeutralMode(NeutralMode.Coast);
        this.d = d;
    }


    @Override
    public void execute() {
        motor.set(TalonSRXControlMode.PercentOutput, dash.get() * d);
    }

    @Override
    public void end(boolean interrupted) {
        motor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }
}
