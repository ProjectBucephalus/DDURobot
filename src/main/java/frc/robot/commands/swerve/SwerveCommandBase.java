package frc.robot.commands.swerve;

import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldUtils;
import frc.robot.util.controlTransmutation.ObjectList;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class SwerveCommandBase extends Command
{
  protected final double deadband = Constants.Control.stickDeadband;

  protected CommandSwerveDrivetrain s_Swerve;

  protected Supplier<Translation2d> joystickSupplier;
  protected Supplier<SwerveDriveState> swerveStateSup;
  
  protected double translationVal;
  protected double strafeVal;
  protected Translation2d motionXY;
  protected double motionXYCache;
  protected Translation2d robotXY;  
  protected double brakeVal;

  protected double robotSpeed;
  protected double robotRadius;
  protected ObjectList fieldGeoFence;
  protected boolean redAlliance;

  /** Creates a new SwerveCommandBase. This has no rotation or drive-request methods or objects */
  public SwerveCommandBase(CommandSwerveDrivetrain s_Swerve, Supplier<Translation2d> joystickSupplier) 
  {
    this.s_Swerve = s_Swerve;
    
    this.joystickSupplier = joystickSupplier;

    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() 
  {
    redAlliance = FieldUtils.isRedAlliance();

    if (redAlliance)
      {fieldGeoFence = FieldConstants.GeoFencing.fieldRedGeoFence;}
    else
      {fieldGeoFence = FieldConstants.GeoFencing.fieldBlueGeoFence;}

    initDriveConstraints();
  }

  /** Override to add additional initialization functionality */
  protected void initDriveConstraints() {}
}
