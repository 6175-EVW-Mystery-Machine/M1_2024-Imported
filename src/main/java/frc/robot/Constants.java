
package frc.robot;

import java.util.List;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.math.VelocityPitchInterpolator;
import frc.robot.math.VelocityPitchInterpolator.ShootingSettings;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public final class Constants {

  public static final class MyConstants {
    //CAN AND CHANNELS
    public static final int kShooterLeftCAN = 11;
    public static final int kShooterRightCAN = 12;
    public static final int kTransferCAN = 14;

    // <<<<< WILL CHANGE >>>>>
    // Wait for new shooter
    public static final int kRotatorCAN = 10;
    public static final int kIntakeCAN = 15;
    public static final int kIntakeForwardChannel = 1;
    public static double ktriggerL;
     public static double ktriggerR;

    public static String colorSensed;
    public static double blinkinTest;

    public static final double TARGET_OFFSET = inchesToMeters(4);


    public static final VelocityPitchInterpolator SHOOTER_INTERPOLATOR = new VelocityPitchInterpolator(List.of(
      //NOT MY INTERPOLATION TABLE
      new ShootingSettings().distance(Meters.of(1.34).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(50)).pitch(Degrees.of(35)),
      new ShootingSettings().distance(Meters.of(1.56).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(50)).pitch(Degrees.of(33)),
      new ShootingSettings().distance(Meters.of(1.922).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(50)).pitch(Degrees.of(26)),
      new ShootingSettings().distance(Meters.of(2.216).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(50)).pitch(Degrees.of(22)),
      new ShootingSettings().distance(Meters.of(2.673).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(50)).pitch(Degrees.of(17)),
      new ShootingSettings().distance(Meters.of(3.442).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(65)).pitch(Degrees.of(9.)),
      new ShootingSettings().distance(Meters.of(4.005).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(80)).pitch(Degrees.of(5)),
      new ShootingSettings().distance(Meters.of(4.445).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(80)).pitch(Degrees.of(3)),
      new ShootingSettings().distance(Meters.of(4.905).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(80)).pitch(Degrees.of(1.25)),
      new ShootingSettings().distance(Meters.of(5.357).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(72)).pitch(Degrees.of(0)),
      new ShootingSettings().distance(Meters.of(5.728).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(67.25)).pitch(Degrees.of(0)),
      new ShootingSettings().distance(Meters.of(6.194).minus(Meters.of(TARGET_OFFSET))).velocity(RotationsPerSecond.of(64.5)).pitch(Degrees.of(0.0))
    ));


    //SPEED VALUES

    //OTHER VALUES AND CONSTANTS
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 29;
    public static final int kRearLeftDrivingCanId = 22;
    public static final int kFrontRightDrivingCanId = 23;
    public static final int kRearRightDrivingCanId = 28;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final double kDrivingMotorPinionTeeth = 15;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 20) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.01;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;

    //################## ENTER VALUES ################
    public static int kTriangleButton = 4;
    public static int kSquareButton = 3;
    public static int kCircleButtion = 2;
    public static int kXButton = 1;

    public static int kLeftBumper = 5;
    public static int kRightBumper = 6;
    public static int kLeftTrigger;
    public static int kRightTrigger; 

    public static int kUpArrowDeg = 0;
    public static int kDownArrowDeg = 180;
    public static int kLeftArrowDeg = 270;
    public static int kRightArrowDeg = 90; 
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}