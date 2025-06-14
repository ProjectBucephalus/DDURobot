package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TargetStationDrive extends HeadingLockedDrive 
{
  /** Creates a new TargetStationDrive. */
  public TargetStationDrive
  (
    CommandSwerveDrivetrain s_Swerve, 
    Supplier<SwerveDriveState> swerveStateSup, 
    DoubleSupplier translationSup, 
    DoubleSupplier strafeSup, 
    Rotation2d rotationOffset, 
    DoubleSupplier brakeSup
  ) 
  {
    super(s_Swerve, swerveStateSup, translationSup, strafeSup, Rotation2d.kZero, rotationOffset, brakeSup);
  }

  @Override
  protected void updateTargetHeading()
  {
    targetHeading = redAlliance ^ (robotXY.getY() >= 4.026) ? 
      new Rotation2d(Units.degreesToRadians(-144)) : // Left side if blue, right side if red
      new Rotation2d(Units.degreesToRadians(144)); // Right side if blue, left side if red
  }
}