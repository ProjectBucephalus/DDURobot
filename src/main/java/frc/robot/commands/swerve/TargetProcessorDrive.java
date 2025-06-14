package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;

public class TargetProcessorDrive extends HeadingLockedDrive 
{
  /** Creates a new TargetProcessorDrive. */
  public TargetProcessorDrive
  (
    CommandSwerveDrivetrain s_Swerve, 
    Supplier<SwerveDriveState> swerveStateSup, 
    DoubleSupplier translationSup, 
    DoubleSupplier strafeSup, 
    Rotation2d targetHeading, 
    Rotation2d rotationOffset, 
    DoubleSupplier brakeSup
  ) 
  {
    super(s_Swerve, swerveStateSup, translationSup, strafeSup, targetHeading, rotationOffset, brakeSup);
  }

  @Override
  protected void updateTargetHeading()
  {
    targetHeading = FieldUtils.isRedAlliance() ? Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg;
  }
}
