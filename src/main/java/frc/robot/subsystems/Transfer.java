package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transfer extends SubsystemBase {
  
  private final CANSparkMax m_transfer;

  public Transfer() {
    m_transfer = new CANSparkMax(Constants.MyConstants.kTransferCAN, MotorType.kBrushless);
    
    m_transfer.restoreFactoryDefaults();  
    m_transfer.setInverted(false);
    m_transfer.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
  
  }
   public void c_runTransfer(double speed) {
  m_transfer.set(speed);
  }

  public Command c_runTransferAuto(double speed) {
  return new InstantCommand(() -> m_transfer.set(speed), this);
  }
}
