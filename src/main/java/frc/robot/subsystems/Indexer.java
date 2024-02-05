package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Indexer extends SubsystemBase {

  private final CANSparkMax m_indexerL;
//private final CANSparkMax m_indexerR;

  public Indexer() {
    m_indexerL = new CANSparkMax(11, MotorType.kBrushless);
//m_indexerR = new CANSparkMax(12, MotorType.kBrushless);

  m_indexerL.restoreFactoryDefaults();  
  m_indexerL.setInverted(false);
  m_indexerL.setIdleMode(IdleMode.kBrake);

  
// m_indexterR.restoreFactoryDefaults();  
// m_indexterR.setInverted(false);
// m_indexterR.setIdleMode(IdleMode.kBrake);
// m_indexterR.burnFlash();
  }

  @Override
  public void periodic() {
  
  }

  public void c_runIndex(double speed) {
    m_indexerL.set(speed);
  //m_indexerR.set(speed);
  }
}
