package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ShootSpeaker extends Command {

public static final Translation2d RED_SPEAKER = new Translation2d(16.57997700, 5.54775544);
public static final Translation2d BLUE_SPEAKER = new Translation2d(0, 5.54775544);
public static final Rotation2d HALF = Rotation2d.fromDegrees(180);
private static final double GAIN = 1/90.0;
private static double m_error = 360;


private final DriveSubsystem m_robotDrive;

  public ShootSpeaker(DriveSubsystem m_robotDrive) {
    this.m_robotDrive = m_robotDrive;
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     var alliance = DriverStation.getAlliance();  
      Translation2d currentTranslation = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red ? RED_SPEAKER : BLUE_SPEAKER;
      // Rotation2d targetAngle = currentTranslation.minus(m_robotDrive.getPose2d().getTranslation()).getAngle().plus(HALF);
       Rotation2d targetAngle = currentTranslation.minus(m_robotDrive.getPose2d().getTranslation()).getAngle();
      Rotation2d error = targetAngle.minus(m_robotDrive.getPose2d().getRotation());
      m_error = error.getDegrees();

    m_robotDrive.drive(0, 0, error.getDegrees()*GAIN, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.drive(0, 0, 0, true, true);  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_error) < 2;
  }
}
