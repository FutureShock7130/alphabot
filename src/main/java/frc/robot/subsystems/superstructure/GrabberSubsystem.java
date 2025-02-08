// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.superstructure;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SoftLimitConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.ctre.phoenix6.hardware.CANcoder;
// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.Timer;

// public class GrabberSubsystem extends SubsystemBase {
//   private final SparkMax angleMotor;
//   private final SparkMax intakeMotor;
//   private final CANcoder encoder;
  
//   private static final double kG = 0.05; // Gravity feed forward - adjust this!
//   private static final double STALL_RPM_THRESHOLD = 100.0; // Adjust based on testing
//   private static final double STALL_TIME_THRESHOLD = 0.2; // 200ms
//   private double stallStartTime = 0.0;
//   private boolean isStalled = false;
  
//   // Shuffleboard entries
//   private final ShuffleboardTab grabberTab = Shuffleboard.getTab("Grabber");
//   private final GenericEntry angleUpButton;
//   private final GenericEntry angleDownButton;
//   private final GenericEntry intakeInButton;
//   private final GenericEntry intakeOutButton;
//   private final GenericEntry angleEntry;
//   private final GenericEntry motorRotationsEntry;
//   private final GenericEntry intakeCurrentEntry;
  
//   /** Creates a new GrabberSubsystem. */
//   public GrabberSubsystem() {
//     angleMotor = new SparkMax(4, MotorType.kBrushless);  // Update ID as needed
//     intakeMotor = new SparkMax(6, MotorType.kBrushless); // Update ID as needed
//     encoder = new CANcoder(5);  // Update ID as needed
    
//     configureNEO(angleMotor);
//     configureNEO550(intakeMotor);
    
//     // Create Shuffleboard entries
//     angleUpButton = grabberTab.add("Angle Up", false)
//         .withWidget("Toggle Button")
//         .withPosition(0, 0)
//         .withSize(1, 1)
//         .getEntry();
        
//     angleDownButton = grabberTab.add("Angle Down", false)
//         .withWidget("Toggle Button")
//         .withPosition(1, 0)
//         .withSize(1, 1)
//         .getEntry();
        
//     intakeInButton = grabberTab.add("Intake In", false)
//         .withWidget("Toggle Button")
//         .withPosition(2, 0)
//         .withSize(1, 1)
//         .getEntry();
        
//     intakeOutButton = grabberTab.add("Intake Out", false)
//         .withWidget("Toggle Button")
//         .withPosition(3, 0)
//         .withSize(1, 1)
//         .getEntry();
        
//     angleEntry = grabberTab.add("Angle (CANcoder)", 0.0)
//         .withPosition(0, 1)
//         .withSize(2, 1)
//         .getEntry();
        
//     motorRotationsEntry = grabberTab.add("Angle Motor Rotations", 0.0)
//         .withPosition(0, 2)
//         .withSize(2, 1)
//         .getEntry();
        
//     intakeCurrentEntry = grabberTab.add("Intake Current", 0.0)
//         .withPosition(2, 1)
//         .withSize(2, 1)
//         .getEntry();
//   }

//   private void configureNEO(SparkMax motor) {
//     SparkMaxConfig neoConfig = new SparkMaxConfig();
    
//     SoftLimitConfig softLimitConfig = new SoftLimitConfig();
//     softLimitConfig
//         .forwardSoftLimit(10.0)    // Adjust these limits!
//         .forwardSoftLimitEnabled(true)
//         .reverseSoftLimit(0.0)
//         .reverseSoftLimitEnabled(true);
    
//     neoConfig
//         .smartCurrentLimit(40)
//         .idleMode(IdleMode.kBrake)
//         .voltageCompensation(12.0)
//         .openLoopRampRate(0.1)
//         .apply(softLimitConfig)
//         .inverted(false);   // Adjust if needed
    
//     motor.setCANTimeout(250);
//     motor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     motor.getEncoder().setPosition(0.0);
//   }

//   private void configureNEO550(SparkMax motor) {
//     SparkMaxConfig neoConfig = new SparkMaxConfig();
    
//     neoConfig
//         .smartCurrentLimit(25)  // Lower current limit for NEO 550
//         .idleMode(IdleMode.kCoast)
//         .voltageCompensation(12.0)
//         .openLoopRampRate(0.1)
//         .inverted(false);   // Adjust if needed
    
//     motor.setCANTimeout(250);
//     motor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//   }

//   public void setAngleSpeed(double speed) {
//     angleMotor.set(speed);  // Simple speed control, no gravity compensation needed
//   }

//   public void up() {
//     setAngleSpeed(0.2);  // Adjust speed as needed
//   }

//   public void down() {
//     setAngleSpeed(-0.2);  // Adjust speed as needed
//   }

//   public void stop() {
//     setAngleSpeed(0.0);
//   }

//   public void setIntakeSpeed(double speed) {
//     intakeMotor.set(speed);
//   }

//   public void intake() {
//     setIntakeSpeed(0.2);  // Adjust speed as needed
//   }

//   public void outtake() {
//     setIntakeSpeed(-0.2);  // Adjust speed as needed
//   }

//   public void stopIntake() {
//     setIntakeSpeed(0.0);
//   }

//   @Override
//   public void periodic() {
//     // Check for intake stall using RPM
//     double currentRPM = Math.abs(intakeMotor.getEncoder().getVelocity());
//     double commandedSpeed = Math.abs(intakeMotor.get());
    
//     if (currentRPM < STALL_RPM_THRESHOLD && commandedSpeed > 0.1) {
//       if (!isStalled) {
//         stallStartTime = Timer.getFPGATimestamp();
//         isStalled = true;
//       } else if (Timer.getFPGATimestamp() - stallStartTime > STALL_TIME_THRESHOLD) {
//         stopIntake();  // Stop motor if stalled for too long
//         isStalled = false;
//       }
//     } else {
//       isStalled = false;
//     }

//     // Handle angle controls
//     if (angleUpButton.getBoolean(false)) {
//       up();
//     } else if (angleDownButton.getBoolean(false)) {
//       down();
//     } else {
//       stop();
//     }

//     // Handle intake controls
//     if (intakeInButton.getBoolean(false)) {
//       intake();
//     } else if (intakeOutButton.getBoolean(false)) {
//       outtake();
//     } else {
//       stopIntake();
//     }

//     // Update telemetry
//     angleEntry.setDouble(encoder.getPosition().getValueAsDouble());
//     motorRotationsEntry.setDouble(angleMotor.getEncoder().getPosition());
//     intakeCurrentEntry.setDouble(intakeMotor.getOutputCurrent());
//   }
// }
