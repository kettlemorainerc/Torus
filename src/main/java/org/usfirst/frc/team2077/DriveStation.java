/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.button.*;
import org.usfirst.frc.team2077.command.*;
import org.usfirst.frc.team2077.common.command.*;
import org.usfirst.frc.team2077.common.control.DriveJoystick;
import org.usfirst.frc.team2077.common.control.DriveStick;
import org.usfirst.frc.team2077.common.control.DriveXboxController;
import org.usfirst.frc.team2077.subsystem.Climbers;
import org.usfirst.frc.team2077.subsystem.Intake;
import org.usfirst.frc.team2077.subsystem.LauncherRotater;
import org.usfirst.frc.team2077.subsystem.swerve.SwerveModule;

import java.util.ArrayList;
import java.util.stream.Collectors;

/**
 * This class is intended to be the center point of defining actions that can be utilized during teleop segments of
 * control. This is where we should define what USB port joysticks should be registered as in `FRC Driver Station`'s usb
 * menu. As well as define what buttons on primary/technical driver's controllers should do what.
 * */
public class DriveStation {
    // Common controller port numbers
    // Joysticks that support rotation
    private static final int DRIVE_JOYSTICK_PORT = 0;
    private static final int DRIVE_XBOX_PORT = 1;
    private static final int FLYSKY_PORT = 2;

    private static final int TECHNICAL_XBOX_PORT = 2;

    // Joysticks that do not support rotation
    private static final int TECHNICAL_JOYSTICK_PORT = 4;
    private static final int NUMPAD_PORT = 5;

    private final DriveXboxController driveStick;
    private final Joystick technicalStick;

    public DriveStation() {
        /** Set the driver's control method this MUST be a {@link DriveStick} implementation */
//        driveStick = getFlysky();
//        driveStick = getJoystick();
        driveStick = getXbox(DRIVE_XBOX_PORT);

        /** Set the technical control method. This can be any {@link Joystick} implementation */
//        technicalStick = getTechnicalJoystick();
//        technicalStick = getXbox(TECHNICAL_XBOX_PORT);
        technicalStick = getNumpad();

        bind();
    }

    /**
     * This method binds any subsystem's default command and bind commands to a user's chosen
     * control method.
     */
    public void bind() {
        RobotHardware hardware = RobotHardware.getInstance();

        hardware.getPosition().setDefaultCommand(new CardinalMovement(driveStick));
        hardware.getHeading().setDefaultCommand(new RotationMovement(driveStick));

        bindDriverControl(driveStick);
        bindTechnicalControl(technicalStick);
    }

    /** Bind primary driver's button commands here */
    private static void bindDriverControl(DriveXboxController primary) {

        new ResetGyro().bind(new JoystickButton(primary, 3));
        new ToggleFieldOriented().bind(new JoystickButton(primary, 6));

//        primary.getRightTriggerAxis()
    }

    /** Bind technical driver button commands here */
    private void bindTechnicalControl(Joystick secondary) {

//        new RunLauncher(5, "fast", 1).bind(new JoystickButton(secondary, 1));
//        new RunLauncher(6, "fast", 1).bind(new JoystickButton(secondary, 1));
//
//        new RunLauncher(5, "in", -1).bind(new JoystickButton(secondary, 5));
//        new RunLauncher(6, "in", -1).bind(new JoystickButton(secondary, 5));
//
//        new RunLauncher(7, "slow", 1).bind(new JoystickButton(secondary, 2));
//        new RunLauncher(8, "slow", 1).bind(new JoystickButton(secondary, 2));
//
//        new RunLauncher(7, "slow", -1).bind(new JoystickButton(secondary, 6));
//        new RunLauncher(8, "slow", -1).bind(new JoystickButton(secondary, 6));

//        swerveVelocityPID(secondary);
//        swerveAnglePID(secondary);

        new RaiseClimber(Climbers.Direction.UP).bind(new JoystickButton(secondary, 1));
        new RaiseClimber(Climbers.Direction.DOWN).bind(new JoystickButton(secondary, 5));

        new RunLauncher(1,1).bind(new JoystickButton(secondary, 0)); //launcher
        new RunLauncher(1,-1).bind(new JoystickButton(secondary, 0)); //intake to load

        new RunIntake().bind(new JoystickButton(secondary, 0));
        //TODO:button num tbd

        new RotateLauncher(LauncherRotater.InputDir.FORWARD).bind(new JoystickButton(secondary, 0));
        new RotateLauncher(LauncherRotater.InputDir.BACKWARD).bind(new JoystickButton(secondary, 0));
        new RotateLauncher(LauncherRotater.InputDir.FRONT).bind(new JoystickButton(secondary, 0));



    }

    private static void swerveVelocityPID(Joystick stick){
        ArrayList<SwerveModule> modules = new ArrayList<>(RobotHardware.getInstance().getChassis().getDriveModules().values());
        ArrayList<SparkMaxPIDController> pids = new ArrayList<>(modules.stream().map(SwerveModule::getDrivingPID).collect(Collectors.toList()));

        new SparkMaxPIDTuner<>(
            modules,
            pids,
            SwerveModule::calibrationSetVelocity,
            SwerveModule::getVelocityMeasured,
            SwerveModule::savePID,
            0.00015, 0.0004,
            2.0, 4,
            SparkMaxPIDTuner.ErrorMethod.DIFFERENCE,
            50,
            new JoystickButton(stick, 2)
        ).bind(new JoystickButton(stick, 1));

    }

    private static void swerveAnglePID(Joystick stick){
        ArrayList<SwerveModule> modules = new ArrayList<>(RobotHardware.getInstance().getChassis().getDriveModules().values());
        ArrayList<SparkMaxPIDController> pids = new ArrayList<>(modules.stream().map(SwerveModule::getGuidingPID).collect(Collectors.toList()));

        new SparkMaxPIDTuner<>(
            modules, pids,
            SwerveModule::calibrationSetAngle,
            SwerveModule::getAngle,
            SwerveModule::savePID,
                0.1124, 0.00003915,
            Math.PI / 2.0, 4,
            SparkMaxPIDTuner.ErrorMethod.ANGLE_DIFFERENCE,
            10,
            new JoystickButton(stick, 2)
        ).bind(new JoystickButton(stick, 1));

    }

    /** Normal (silver/brighter) joystick that supports rotation */
    private static DriveJoystick getJoystick() {
        return new DriveJoystick(DRIVE_JOYSTICK_PORT).setDriveSensitivity(.15, 5)
                                                     .setRotationSensitivity(.1, 1);
    }

    /** Flysky Drone Controller */
    private static DriveJoystick getFlysky() {
        return new DriveJoystick(FLYSKY_PORT, 4).setDriveSensitivity(.3, 1)
                                                .setRotationSensitivity(.05, 2.5);
    }

    private static DriveXboxController getXbox(int port){
        return new DriveXboxController(port).setDriveSensitivity(.3,1)
                                                       .setRotationSensitivity(.3,1.5);
    }

    /** Currently the darker joystick that doesn't support rotation */
    private static Joystick getTechnicalJoystick() {
        return new Joystick(TECHNICAL_JOYSTICK_PORT);
    }

    private static Joystick getNumpad() {
        return new Joystick(NUMPAD_PORT);
    }

    /** bind command to the given joystick button */
    public static void useCommand(Joystick joystick, int button, BindableCommand command) {
        command.bind(new JoystickButton(joystick, button));
    }
}
