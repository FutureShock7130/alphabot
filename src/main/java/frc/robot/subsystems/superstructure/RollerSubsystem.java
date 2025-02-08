// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.spark.config.SoftLimitConfig;

public class RollerSubsystem extends SubsystemBase {
    private final SparkMax angleMotor;
    private final SparkMax intakeMotor;
    private final CANcoder encoder;

    // Shuffleboard entries
    private final ShuffleboardTab rollerTab = Shuffleboard.getTab("Roller");
    private final GenericEntry deployButton;
    private final GenericEntry retractButton;
    private final GenericEntry intakeInButton;
    private final GenericEntry intakeOutButton;
    private final GenericEntry angleEntry;
    private final GenericEntry motorRotationsEntry;
    private final GenericEntry intakeRPMEntry;

    private static final double STALL_RPM_THRESHOLD = 100.0;
    private static final double STALL_TIME_THRESHOLD = 0.2;
    private double stallStartTime = 0.0;
    private boolean isStalled = false;

    /** Creates a new RollerSubsystem. */
    public RollerSubsystem() {
        angleMotor = new SparkMax(7, MotorType.kBrushless); // Update ID as needed
        intakeMotor = new SparkMax(8, MotorType.kBrushless); // Update ID as needed
        encoder = new CANcoder(9); // Update ID as needed

        configureNEO(angleMotor);
        configureNEO550(intakeMotor);

        // Create Shuffleboard entries
        deployButton = rollerTab.add("Deploy Roller", false)
                .withWidget("Toggle Button")
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();

        retractButton = rollerTab.add("Retract Roller", false)
                .withWidget("Toggle Button")
                .withPosition(1, 0)
                .withSize(1, 1)
                .getEntry();

        intakeInButton = rollerTab.add("Roller In", false)
                .withWidget("Toggle Button")
                .withPosition(2, 0)
                .withSize(1, 1)
                .getEntry();

        intakeOutButton = rollerTab.add("Roller Out", false)
                .withWidget("Toggle Button")
                .withPosition(3, 0)
                .withSize(1, 1)
                .getEntry();

        angleEntry = rollerTab.add("Angle (CANcoder)", 0.0)
                .withPosition(0, 1)
                .withSize(2, 1)
                .getEntry();

        motorRotationsEntry = rollerTab.add("Angle Motor Rotations", 0.0)
                .withPosition(0, 2)
                .withSize(2, 1)
                .getEntry();

        intakeRPMEntry = rollerTab.add("Intake RPM", 0.0)
                .withPosition(2, 1)
                .withSize(2, 1)
                .getEntry();
    }

    private void configureNEO(SparkMax motor) {
        SparkMaxConfig neoConfig = new SparkMaxConfig();

        // Create soft limit config for roller
        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig
                .forwardSoftLimit(3) // Adjust these limits for your roller!
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(0.0) // Bottom position
                .reverseSoftLimitEnabled(true);

        neoConfig
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12.0)
                .openLoopRampRate(0.1)
                .apply(softLimitConfig) // Apply the soft limits
                .inverted(false); // Adjust if needed

        motor.setCANTimeout(250);
        motor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.getEncoder().setPosition(0.0); // Reset encoder to zero
    }

    private void configureNEO550(SparkMax motor) {
        SparkMaxConfig neoConfig = new SparkMaxConfig();

        neoConfig
                .smartCurrentLimit(25)
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(12.0)
                .openLoopRampRate(0.1)
                .inverted(false); // Adjust if needed

        motor.setCANTimeout(250);
        motor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setAngleSpeed(double speed) {
        angleMotor.set(speed);
    }

    public void deploy() {
        setAngleSpeed(0.2); // Adjust speed as needed
    }

    public void retract() {
        setAngleSpeed(-0.2); // Adjust speed as needed
    }

    public void stopAngle() {
        setAngleSpeed(0.0);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void intake() {
        setIntakeSpeed(0.2); // Adjust speed as needed
    }

    public void outtake() {
        setIntakeSpeed(-0.2); // Adjust speed as needed
    }

    public void stopIntake() {
        setIntakeSpeed(0.0);
    }

    @Override
    public void periodic() {
        // Check for intake stall using RPM
        double currentRPM = Math.abs(intakeMotor.getEncoder().getVelocity());
        double commandedSpeed = Math.abs(intakeMotor.get());

        if (currentRPM < STALL_RPM_THRESHOLD && commandedSpeed > 0.1) {
            if (!isStalled) {
                stallStartTime = Timer.getFPGATimestamp();
                isStalled = true;
            } else if (Timer.getFPGATimestamp() - stallStartTime > STALL_TIME_THRESHOLD) {
                stopIntake();
                isStalled = false;
            }
        } else {
            isStalled = false;
        }

        // Handle angle controls
        if (deployButton.getBoolean(false)) {
            deploy();
        } else if (retractButton.getBoolean(false)) {
            retract();
        } else {
            stopAngle();
        }

        // Handle intake controls
        if (intakeInButton.getBoolean(false)) {
            intake();
        } else if (intakeOutButton.getBoolean(false)) {
            outtake();
        } else {
            stopIntake();
        }

        // Update telemetry
        angleEntry.setDouble(encoder.getPosition().getValueAsDouble());
        motorRotationsEntry.setDouble(angleMotor.getEncoder().getPosition());
        intakeRPMEntry.setDouble(currentRPM);
    }
}
