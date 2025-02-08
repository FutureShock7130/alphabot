// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.controller.PIDController;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    // private final CANcoder encoder;

    private static final double kG = 0.05; // Gravity feed forward constant - adjust this!
    private static final double kDownSpeedMultiplier = 0.5; // Reduces down speed - adjust this!

    // Shuffleboard entries
    private final ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");
    private final GenericEntry upButton = elevatorTab.add("Elevator Up", false)
            .withWidget("Toggle Button")
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();

    private final GenericEntry downButton = elevatorTab.add("Elevator Down", false)
            .withWidget("Toggle Button")
            .withPosition(1, 0)
            .withSize(1, 1)
            .getEntry();

    // Add these with other instance variables at the top
    private final GenericEntry speedEntry;
    private final GenericEntry positionEntry;
    private final GenericEntry leftRotationsEntry;
    private final GenericEntry rightRotationsEntry;

    // Add these variables at the top
    private double maxLeftRotations = 0.0;
    private double maxRightRotations = 0.0;
    private final GenericEntry maxLeftRotationsEntry;
    private final GenericEntry maxRightRotationsEntry;

    private final PIDController pidController = new PIDController(0.07, 0.0, 0.0); // Adjust P, I, D values!
    private boolean positionLocked = false;
    private double targetPosition = 0.0;
    private final GenericEntry lockButton;

    /** Creates a new ElevatorSubsystem. */
    public ElevatorSubsystem() {
        leftMotor = new SparkMax(1, MotorType.kBrushless); // Update ID as needed
        rightMotor = new SparkMax(2, MotorType.kBrushless); // Update ID as needed
        // encoder = new CANcoder(3); // Update CANcoder ID as needed!

        configureNEO(leftMotor, true); // Invert both motors to make default direction clockwise
        configureNEO(rightMotor, true); // Both motors still turn the same way

        // Set position conversion factor for encoder

        // Create all widgets once in constructor
        speedEntry = elevatorTab.add("Elevator Speed", 0.0)
                .withPosition(0, 1)
                .withSize(2, 1)
                .getEntry();
        positionEntry = elevatorTab.add("Elevator Position (CANcoder)", 0.0)
                .withPosition(0, 2)
                .withSize(2, 1)
                .getEntry();
        leftRotationsEntry = elevatorTab.add("Left Motor Rotations", 0.0)
                .withPosition(0, 3)
                .withSize(2, 1)
                .getEntry();
        rightRotationsEntry = elevatorTab.add("Right Motor Rotations", 0.0)
                .withPosition(0, 4)
                .withSize(2, 1)
                .getEntry();

        maxLeftRotationsEntry = elevatorTab.add("Max Left Motor Rotations", 0.0)
                .withPosition(0, 5)
                .withSize(2, 1)
                .getEntry();
        maxRightRotationsEntry = elevatorTab.add("Max Right Motor Rotations", 0.0)
                .withPosition(0, 6)
                .withSize(2, 1)
                .getEntry();

        lockButton = elevatorTab.add("Lock Position", false)
                .withWidget("Toggle Button")
                .withPosition(2, 0)
                .withSize(1, 1)
                .getEntry();

        // or
    }

    private void configureNEO(SparkMax motor, boolean inverted) {
        SparkMaxConfig neoConfig = new SparkMaxConfig();

        // Create soft limit config for elevator
        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig
                .forwardSoftLimit(28) // Adjust these limits for your elevator!
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(0.0) // Bottom position
                .reverseSoftLimitEnabled(true);

        neoConfig
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake) // Use brake mode for elevator
                .voltageCompensation(12.0)
                .openLoopRampRate(0.1)
                .apply(softLimitConfig)
                .inverted(inverted);

        // Add follow configuration for right motor
        if (motor == rightMotor) {
            neoConfig.follow(leftMotor);
        }

        motor.setCANTimeout(250);
        motor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.getEncoder().setPosition(0.0); // Reset encoder to zero
    }

    /**
     * Sets the elevator speed. Positive values move up, negative values move down.
     * Includes gravity compensation when moving up and speed reduction when moving
     * down! ^w^
     * 
     * @param speed Speed from -1.0 to 1.0
     */
    public void setElevatorSpeed(double speed) {
        // Add gravity feedforward when moving up
        double gravityCompensation = (speed >= 0) ? kG : 0.0;

        // Reduce speed when moving down
        if (speed < 0) {
            speed *= kDownSpeedMultiplier;
        }

        leftMotor.set(speed + gravityCompensation); // Right motor follows automatically uwu
    }

    /**
     * Move the elevator up at a fixed speed
     */
    public void up() {
        setElevatorSpeed(0.1); // Adjust this value based on your needs!
    }

    /**
     * Move the elevator down at a fixed speed
     */
    public void down() {
        setElevatorSpeed(-0.1); // Adjust this value based on your needs!
    }

    /**
     * Stop the elevator
     */
    public void stop() {
        setElevatorSpeed(0.0);
    }

    public void lockPosition() {
        positionLocked = true;
        // targetPosition = encoder.getPosition().getValueAsDouble(); // cancoder
        targetPosition = leftMotor.getEncoder().getPosition(); // encoder
    }

    public void unlockPosition() {
        positionLocked = false;
    }

    @Override
    public void periodic() {
        // Check for negative position and reset if needed
        if (leftMotor.getEncoder().getPosition() < 0) {
            leftMotor.getEncoder().setPosition(0.0);
        }
        if (rightMotor.getEncoder().getPosition() < 0) {
            rightMotor.getEncoder().setPosition(0.0);
        }

        // Handle position locking
        if (lockButton.getBoolean(false)) {
            if (!positionLocked) {
                lockPosition();
            }
            // double currentPosition = encoder.getPosition().getValueAsDouble(); //cancoder
            double currentPosition = leftMotor.getEncoder().getPosition(); // encoder
            double output = pidController.calculate(currentPosition, targetPosition);
            setElevatorSpeed(output);
        } else {
            if (positionLocked) {
                unlockPosition();
            }
            // Normal button control
            if (upButton.getBoolean(false)) {
                up();
            } else if (downButton.getBoolean(false)) {
                down();
            } else {
                stop();
            }
        }

        // Update values instead of creating new widgets
        speedEntry.setDouble(leftMotor.get());
        // positionEntry.setDouble(encoder.getPosition().getValueAsDouble()); //cancoder
        positionEntry.setDouble(leftMotor.getEncoder().getPosition()); // encoder
        leftRotationsEntry.setDouble(leftMotor.getEncoder().getPosition());
        rightRotationsEntry.setDouble(rightMotor.getEncoder().getPosition());

        // Track maximum rotations
        double leftRotations = Math.abs(leftMotor.getEncoder().getPosition());
        double rightRotations = Math.abs(rightMotor.getEncoder().getPosition());

        maxLeftRotations = Math.max(maxLeftRotations, leftRotations);
        maxRightRotations = Math.max(maxRightRotations, rightRotations);

        // Update max rotation displays
        maxLeftRotationsEntry.setDouble(maxLeftRotations);
        maxRightRotationsEntry.setDouble(maxRightRotations);
    }
}
