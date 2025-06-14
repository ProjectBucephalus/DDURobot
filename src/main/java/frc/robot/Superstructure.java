package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.ManualDrive;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.*;

public class Superstructure 
{
  /* Driver Control Axis */
  public static final int translationAxis = Axis.kLeftY.value;
  public static final int strafeAxis      = Axis.kLeftX.value;
  public static final int rotationAxis    = Axis.kRightX.value;
  public static final int brakeAxis       = Axis.kRightTrigger.value;

  private SwerveDriveState swerveState;
  private Field2d field;
  private final CommandSwerveDrivetrain s_Swerve;
  private final CoralRoller s_Coral;
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController copilot = new CommandXboxController(1);

  public Superstructure()
  {
    field = new Field2d();
    s_Swerve = TunerConstants.createDrivetrain();
    s_Coral = new CoralRoller();

    SmartDashboard.putData("Field", field);

    bindControls();
  }

  public SwerveDriveState getSwerveState() 
    {return swerveState;}

  public void updateSwerveState()
  {
    swerveState = s_Swerve.getState();
    field.setRobotPose(swerveState.Pose);
  }

  private void bindControls()
  {
    s_Swerve.setDefaultCommand
    (
      new ManualDrive
      (
        s_Swerve, 
        this::getSwerveState, 
        () -> -driver.getRawAxis(translationAxis), 
        () -> -driver.getRawAxis(strafeAxis), 
        () -> -driver.getRawAxis(rotationAxis), 
        () -> driver.getRawAxis(brakeAxis)
      )
    );

    copilot.leftTrigger().whileTrue(s_Coral.smartRunCommand(Constants.Coral.forwardSpeed));
    copilot.rightTrigger().whileTrue(s_Coral.smartRunCommand(Constants.Coral.reverseSpeed));
  }
}
