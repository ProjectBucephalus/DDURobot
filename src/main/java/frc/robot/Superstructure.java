package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.*;

public class Superstructure 
{
  private SwerveDriveState swerveState;
  private Field2d field;
  private final CommandSwerveDrivetrain s_Swerve;
  private final CoralRoller s_Coral;
  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController copilot = new CommandXboxController(1);

  public Superstructure()
  {
    field = new Field2d();
    s_Swerve = TunerConstants.createDrivetrain();
    s_Coral = new CoralRoller();

    SmartDashboard.putData("Field", field);
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
    copilot.leftTrigger().whileTrue(s_Coral.smartRunCommand(Constants.Coral.forwardSpeed));
    copilot.rightTrigger().whileTrue(s_Coral.smartRunCommand(Constants.Coral.reverseSpeed));
  }
}
