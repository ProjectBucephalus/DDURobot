package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;
import frc.robot.util.controlTransmutation.CrossDeadband;
import frc.robot.util.controlTransmutation.Deadband;

public class TargetScoreDrive extends HeadingLockedDrive 
{
  private Rotation2d rotationOffsetBase;
  private int nearestReefFace;
  private Deadband crossDeadband = new CrossDeadband();

  /** Creates a new TargetScoreDrive. */
  public TargetScoreDrive
  (
    CommandSwerveDrivetrain s_Swerve, 
    Supplier<Translation2d> joystickSupplier,
    Rotation2d rotationOffset,
    Supplier<Translation2d> robotPosSup
  ) 
  {
    super(s_Swerve, joystickSupplier, Rotation2d.kZero, rotationOffset, robotPosSup);
    this.rotationOffsetBase = rotationOffset;
  }

  @Override
  protected void updateTargetHeading()
  { 
    if (MathUtil.isNear(robotXY.getX(), (FieldConstants.fieldCentre.getX()), Constants.Control.driveSnappingRange)) 
      {motionXY = crossDeadband.process(motionXY);}

    if 
    (
      FieldConstants.GeoFencing.reefBlue.getDistance() >= FieldConstants.GeoFencing.robotRadiusCircumscribed &&
      FieldConstants.GeoFencing.reefRed.getDistance() >= FieldConstants.GeoFencing.robotRadiusCircumscribed
    )
    {
      nearestReefFace = FieldUtils.getNearestReefFace(robotXY);

      switch (nearestReefFace) 
      {
        case 1:
          targetHeading = Rotation2d.kZero;
          super.rotationOffset = this.rotationOffsetBase;
          break;

        case 2:
          targetHeading = new Rotation2d(Units.degreesToRadians(60));
          super.rotationOffset = this.rotationOffsetBase.unaryMinus();
          break;

        case 3:
          targetHeading = new Rotation2d(Units.degreesToRadians(120));
          super.rotationOffset = this.rotationOffsetBase.unaryMinus();
          break;

        case 4:
          targetHeading = Rotation2d.k180deg;
          super.rotationOffset = this.rotationOffsetBase.unaryMinus();
          break;

        case 5:
          targetHeading = new Rotation2d(Units.degreesToRadians(-120));
          super.rotationOffset = this.rotationOffsetBase.unaryMinus();
          break;

        case 6:
          targetHeading = new Rotation2d(Units.degreesToRadians(-60));
          super.rotationOffset = this.rotationOffsetBase.unaryMinus();
          break;
          
        default:
          break;
      }
    }
  }
}