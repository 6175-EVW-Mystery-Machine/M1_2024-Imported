package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rotator extends SubsystemBase {

//   private final CANSparkFlex m_rotatorMotor;

//   // private RelativeEncoder rotEncoder;
//   // private SparkPIDController m_pidController;
//   // public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

public Rotator() {

//   m_rotatorMotor = new CANSparkFlex(Constants.MyConstants.kRotatorCAN, MotorType.kBrushless);

//   m_rotatorMotor.restoreFactoryDefaults();  
//   m_rotatorMotor.setInverted(false);
//   m_rotatorMotor.setIdleMode(IdleMode.kBrake);
//   m_rotatorMotor.burnFlash();

//   // rotEncoder = m_rotatorMotor.getEncoder();
//   // m_pidController = m_rotatorMotor.getPIDController();

//   //     // PID coefficients
//   //     kP = 0.1; 
//   //     kI = 1e-4;
//   //     kD = 1; 
//   //     kIz = 0; 
//   //     kFF = 0; 
//   //     kMaxOutput = 1; 
//   //     kMinOutput = -1;

//   //   // set PID coefficients
//   //   m_pidController.setP(kP);
//   //   m_pidController.setI(kI);
//   //   m_pidController.setD(kD);
//   //   m_pidController.setIZone(kIz);
//   //   m_pidController.setFF(kFF);
//   //   m_pidController.setOutputRange(kMinOutput, kMaxOutput);

//   //   // display PID coefficients on SmartDashboard
//   //   SmartDashboard.putNumber("P Gain", kP);
//   //   SmartDashboard.putNumber("I Gain", kI);
//   //   SmartDashboard.putNumber("D Gain", kD);
//   //   SmartDashboard.putNumber("I Zone", kIz);
//   //   SmartDashboard.putNumber("Feed Forward", kFF);
//   //   SmartDashboard.putNumber("Max Output", kMaxOutput);
//   //   SmartDashboard.putNumber("Min Output", kMinOutput);
//   //   SmartDashboard.putNumber("Set Rotations", 0);

}

@Override
public void periodic() {
//   //   // read PID coefficients from SmartDashboard
//   //   double p = SmartDashboard.getNumber("P Gain", 0);
//   //   double i = SmartDashboard.getNumber("I Gain", 0);
//   //   double d = SmartDashboard.getNumber("D Gain", 0);
//   //   double iz = SmartDashboard.getNumber("I Zone", 0);
//   //   double ff = SmartDashboard.getNumber("Feed Forward", 0);
//   //   double max = SmartDashboard.getNumber("Max Output", 0);
//   //   double min = SmartDashboard.getNumber("Min Output", 0);

//   //   // if PID coefficients on SmartDashboard have changed, write new values to controller
//   //   if((p != kP)) { m_pidController.setP(p); kP = p; }
//   //   if((i != kI)) { m_pidController.setI(i); kI = i; }
//   //   if((d != kD)) { m_pidController.setD(d); kD = d; }
//   //   if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
//   //   if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
//   //   if((max != kMaxOutput) || (min != kMinOutput)) { 
//   //     m_pidController.setOutputRange(min, max); 
//   //     kMinOutput = min; kMaxOutput = max; 
}
}

//   // public void rotToDeg(int deg) {
//   //   int pos;
//   //   pos = deg/360;
    
//   //   m_pidController.setReference(pos, ControlType.kPosition);
//   // }

//   // public void rotToPos(int pos) {
//   //   //POS is just the number of rotations
//   //   m_pidController.setReference(pos, ControlType.kPosition);
//   public void c_rotJog(double speed) {
//     m_rotatorMotor.set(speed);
//   }

//   }
  
 

// }
