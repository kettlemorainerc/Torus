package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Subsystem;

    public class LauncherRotater implements Subsystem {
        //TODO: determine device number & speed
        private TalonSRX RotationControllerMotor = new TalonSRX(0);
        private TalonSRX RotaryEncoder = new TalonSRX(0);
        private double currentAngle;
        private double targetAngle;
        private double speed;

        public LauncherRotater() {

        }
        @Override
        public void periodic() {
            currentAngle = RotaryEncoder.getSensorCollection().getPulseWidthPosition();
        }
        public void GetTargetAngle() {

        }

        public void GoToTarget(){

        }

        public void RotateToIntake() {

        }


    }
