package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;

public class TargetCageDrive extends HeadingLockedDrive
{
  public TargetCageDrive
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
    rotationOffset = 
    MathUtil.isNear(robotXY.getX(), (FieldUtils.fieldLength / 2), Constants.Control.cageFaceDistance) ? 
      Rotation2d.kZero :
      Rotation2d.kCW_90deg;
  }

  @Override
  protected void applyTranslationDeadband() 
  {    
    if (motionXY.getNorm() <= deadband) {motionXY = Translation2d.kZero;}
  }

  @Override
  protected void updateRobotRadius()
  {
    if (MathUtil.isNear(robotXY.getX(), (FieldUtils.fieldLength / 2), Constants.Control.driveSnappingRange))
      {robotRadius = FieldUtils.GeoFencing.robotRadiusMinimum;}
    else if (robotSpeed >= FieldUtils.GeoFencing.robotSpeedThreshold)
      {robotRadius = FieldUtils.GeoFencing.robotRadiusCircumscribed;}
    else
      {robotRadius = FieldUtils.GeoFencing.robotRadiusInscribed;}
  }
}
