// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.beans.Encoder;

// import com.revrobotics.CANSparkFlex;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.DutyCycle;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
// import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class ShooterPitch2 extends ProfiledPIDSubsystem {
  
//   private final CANSparkFlex m_motor = new CANSparkFlex(10, MotorType.kBrushless);
//   private final DutyCycleEncoder m_encoder =
//     new DutyCycleEncoder(3);
//   private final ArmFeedforward m_feedForward = new ArmFeedforward(0, 0, 0);


//   public ShooterPitch2() {
//     super(
//       new ProfiledPIDController(
//      0.5, 0, 0, new TrapezoidProfile.Constraints(0.5, 0.25)),
//     0);
//     m_encoder.setDistancePerRotation(0.1);
//   }


//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   @Override
//   public void useOutput(double output, TrapezoidProfile.State setpoint) {
//     double feedforward = m_feedForward.calculate(setpoint.position, setpoint.velocity);
//     m_motor.setVoltage(output + feedforward);
//   }

//   @Override
//   protected double getMeasurement() {
//     return  m_encoder.getDistance();
//   }

// }
