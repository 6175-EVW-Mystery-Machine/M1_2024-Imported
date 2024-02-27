// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Sweeper extends SubsystemBase {

  
//   private final CANSparkMax m_sweeperDriver;
//   private final DoubleSolenoid m_sweeperSolenoid;

//   public Sweeper() {
//     //NEED TO ADJUST CAN
//     m_sweeperDriver = new CANSparkMax(25, MotorType.kBrushless);
//     m_sweeperSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 4);
//   }

//   @Override
//   public void periodic() { }

//   public void c_sweeperControl(double speed, Boolean isExtended) {
   

//     if (isExtended == true) {
//       m_sweeperSolenoid.set(DoubleSolenoid.Value.kForward);
//     }
//     if (isExtended == false) {
//       m_sweeperSolenoid.set(DoubleSolenoid.Value.kReverse);
//     }
//     else {
//       m_sweeperSolenoid.set(DoubleSolenoid.Value.kOff);
//     }

//     m_sweeperDriver.set(speed);
// }

// public void c_controlSweeperAxis(double axis, double speed, boolean isExtended) {
//   if (axis >= 0.5) {
//   m_sweeperDriver.set(speed);
//     if (isExtended == true) {
//       m_sweeperSolenoid.set(DoubleSolenoid.Value.kForward);
//     }
//     if (isExtended == false) {
//       m_sweeperSolenoid.set(DoubleSolenoid.Value.kReverse);
//     }
//     else {
//       m_sweeperSolenoid.set(DoubleSolenoid.Value.kOff);
//     }
//   } else {
//   m_sweeperDriver.set(0);
//   m_sweeperSolenoid.set(DoubleSolenoid.Value.kReverse);

//   }
// }

// }