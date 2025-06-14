package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.util.SD;

public class Superstructure 
{
  private SwerveDriveState swerveState;
  private Field2d field;
  private final CommandSwerveDrivetrain s_Swerve;
  private final CoralRoller s_Coral;
  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController copilot = new CommandXboxController(1);
  public static final RumbleRequester io_driverRight   = new RumbleRequester(driver, RumbleType.kRightRumble, SD.RUMBLE_D_R::put, SD.IO_RUMBLE_D::get);
  public static final RumbleRequester io_driverLeft    = new RumbleRequester(driver, RumbleType.kLeftRumble, SD.RUMBLE_D_L::put, SD.IO_RUMBLE_D::get);
  public static final RumbleRequester io_copilotRight  = new RumbleRequester(copilot, RumbleType.kRightRumble, SD.RUMBLE_C_R::put, SD.IO_RUMBLE_C::get);
  public static final RumbleRequester io_copilotLeft   = new RumbleRequester(copilot, RumbleType.kLeftRumble, SD.RUMBLE_C_L::put, SD.IO_RUMBLE_C::get);

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
    copilot.leftBumper().whileTrue(s_Coral.runCommand(Constants.Coral.forwardSpeed));
    copilot.rightBumper().whileTrue(s_Coral.runCommand(Constants.Coral.reverseSpeed));
  }

  private void bindRumbles()
  {
    io_copilotLeft.addRumbleTrigger("coral held", new Trigger(s_Coral::getSensor));
  }
}
