// package frc.robot.commands;

// import java.util.List;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.ShooterPitch;
// import frc.robot.Constants.*;
// import frc.robot.math.VelocityPitchInterpolator;
// import frc.robot.math.VelocityPitchInterpolator.ShootingSettings;
// import static edu.wpi.first.math.util.Units.inchesToMeters;
// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.DegreesPerSecond;
// import static edu.wpi.first.units.Units.FeetPerSecond;
// import static edu.wpi.first.units.Units.Inches;
// import static edu.wpi.first.units.Units.Meters;
// import static edu.wpi.first.units.Units.MetersPerSecond;
// import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
// import static edu.wpi.first.units.Units.RotationsPerSecond;


// public class LinearInterpolator extends Command {

// public static final Translation2d RED_SPEAKER = new Translation2d(16.57997700, 5.54775544);
// public static final Translation2d BLUE_SPEAKER = new Translation2d(0, 5.54775544);
// public static final Rotation2d HALF = Rotation2d.fromDegrees(180);
// private static final double GAIN = 1.0/90.0;
// private static double m_error = 360;

// public static final double TARGET_OFFSET = inchesToMeters(4);


//     public static final VelocityPitchInterpolator SHOOTER_INTERPOLATOR = new VelocityPitchInterpolator(List.of(
//       //NOT MY INTERPOLATION TABLE
//       new ShootingSettings().distance(Meters.of(1.34).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(50)).pitch(Degrees.of(35)),
//       new ShootingSettings().distance(Meters.of(1.56).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(50)).pitch(Degrees.of(33)),
//       new ShootingSettings().distance(Meters.of(1.922).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(50)).pitch(Degrees.of(26)),
//       new ShootingSettings().distance(Meters.of(2.216).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(50)).pitch(Degrees.of(22)),
//       new ShootingSettings().distance(Meters.of(2.673).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(50)).pitch(Degrees.of(17)),
//       new ShootingSettings().distance(Meters.of(3.442).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(65)).pitch(Degrees.of(9.)),
//       new ShootingSettings().distance(Meters.of(4.005).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(80)).pitch(Degrees.of(5)),
//       new ShootingSettings().distance(Meters.of(4.445).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(80)).pitch(Degrees.of(3)),
//       new ShootingSettings().distance(Meters.of(4.905).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(80)).pitch(Degrees.of(1.25)),
//       new ShootingSettings().distance(Meters.of(5.357).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(72)).pitch(Degrees.of(0)),
//       new ShootingSettings().distance(Meters.of(5.728).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(67.25)).pitch(Degrees.of(0)),
//       new ShootingSettings().distance(Meters.of(6.194).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(64.5)).pitch(Degrees.of(0.0))
//     ));


// private final DriveSubsystem m_robotDrive;
// private final  Shooter m_shooter;

// private Translation2d speakerTranslation;

//   public LinearInterpolator(DriveSubsystem m_robotDrive, Shooter m_shooter) {
//     this.m_robotDrive = m_robotDrive;
//     this.m_shooter = m_shooter;
//     addRequirements(m_robotDrive, m_shooter);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     var alliance = DriverStation.getAlliance(); 
//     speakerTranslation = (alliance.isEmpty() || alliance.get() == DriverStation.Alliance.Blue) ? BLUE_SPEAKER : RED_SPEAKER;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_shooter.c_startFlywheel(0.7, 0.7);

//      var alliance = DriverStation.getAlliance();  
//       Translation2d currentTranslation = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red ? RED_SPEAKER : BLUE_SPEAKER;
//       // Rotation2d targetAngle = currentTranslation.minus(m_robotDrive.getPose2d().getTranslation()).getAngle().plus(HALF);
//       Rotation2d targetAngle = currentTranslation.minus(m_robotDrive.getPose2d().getTranslation()).getAngle();
//       Rotation2d error = targetAngle.minus(m_robotDrive.getPose2d().getRotation());
//       m_error = error.getDegrees();

//     m_robotDrive.drive(0, 0, error.getDegrees()*GAIN, true, true);

//     m_shooter.c_startFlywheel(0.7, 0.7);
    
//     var robotPose = m_robotDrive.getPose();
//     var robotTranslation = robotPose.getTranslation();

//     var distanceToSpeaker = robotTranslation.getDistance(speakerTranslation);

//     var shootingSettings = SHOOTER_INTERPOLATOR.calculate(distanceToSpeaker);

//     var angleToSpeaker = speakerTranslation.minus(robotTranslation).getAngle();

//     ShooterPitch.RotToSPGrav(shootingSettings.getPitch());

//     var isPitchReady = ShooterPitch.isAtPitchTarget();

//     if (isPitchReady) {
//       // Shooter is spun up, drivetrain is aimed, robot is stopped, and the turret is aimed - shoot and start timer
//       turretSubsystem.shoot();
//       isShooting = true;
//     }
//   } 

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_robotDrive.drive(0, 0, 0, true, true);  
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return Math.abs(m_error) < 2;
//   }
// }
