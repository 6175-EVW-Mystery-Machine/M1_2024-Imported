// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Rotator;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Transfer;

// public class TestAuto extends SequentialCommandGroup {

//   public TestAuto(DriveSubsystem m_drive, Shooter m_shooter, Rotator m_rotator, Transfer m_transfer, Intake m_intake) {
//     addCommands(

//     new StartFlywheels().withTimeout(1.5),
//     new FeedShooter().withTimeout(1),
//     new DriveForward().withTimeout(1.5),
//     new RunIntake().withTimeout(2.5),
//     new DriveBackward().withTimeout(1.5),
//     new FeedShooter().withTimeout(1),
//     new DriveForward().withTimeout(2) 


//     );
//   }




// }
