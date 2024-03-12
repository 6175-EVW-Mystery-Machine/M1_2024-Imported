// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkFlex;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class RotatorJog extends SubsystemBase {

//   private final CANSparkFlex m_rotJog;

//   public RotatorJog() {
//    // m_rotJog = new CANSparkFlex(Constants.MyConstants.kRotatorCAN, MotorType.kBrushless);
// m_rotJog = new CANSparkFlex(30, MotorType.kBrushless);

//     m_rotJog.restoreFactoryDefaults();  
//     m_rotJog.setInverted(false);
//     m_rotJog.setIdleMode(IdleMode.kBrake);
//     m_rotJog.burnFlash();
//   }

//   @Override
//   public void periodic() {
  
//   }

//   public void c_rotJog(double speed) {
//    m_rotJog.set(speed);
//   }
// }
