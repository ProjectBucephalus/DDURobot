package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.ManualDrive;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.RumbleRequester;
import frc.robot.util.AutoFactories;
import frc.robot.util.FieldUtils;
import frc.robot.util.SD;

public class Superstructure 
{
  /* Driver Control Axes */
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
  private final RumbleRequester io_driverRight   = new RumbleRequester(driver, RumbleType.kRightRumble, SD.RUMBLE_D_R::put, SD.IO_RUMBLE_D::get);
  private final RumbleRequester io_driverLeft    = new RumbleRequester(driver, RumbleType.kLeftRumble, SD.RUMBLE_D_L::put, SD.IO_RUMBLE_D::get);
  private final RumbleRequester io_copilotRight  = new RumbleRequester(copilot, RumbleType.kRightRumble, SD.RUMBLE_C_R::put, SD.IO_RUMBLE_C::get);
  private final RumbleRequester io_copilotLeft   = new RumbleRequester(copilot, RumbleType.kLeftRumble, SD.RUMBLE_C_L::put, SD.IO_RUMBLE_C::get);

  public Superstructure()
  {
    field = new Field2d();
    s_Swerve = TunerConstants.createDrivetrain();
    s_Coral = new CoralRoller();

    updateSwerveState();

    SmartDashboard.putData("Field", field);

    bindControls();
    bindRumbles();
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

    copilot.leftTrigger().whileTrue(s_Coral.smartRunCommand(SD.IO_CORALSPEED_F.get()));
    copilot.rightTrigger().whileTrue(s_Coral.smartRunCommand(SD.IO_CORALSPEED_R.get()));
    copilot.leftBumper().whileTrue(s_Coral.runCommand(SD.IO_ALGAESPEED_F.get()));
    copilot.rightBumper().whileTrue(s_Coral.runCommand(SD.IO_ALGAESPEED_R.get()));
    new Trigger(() -> FieldUtils.atReefLineUp(getSwerveState().Pose.getTranslation())).whileTrue(s_Coral.smartRunCommand(Constants.Coral.forwardSpeed));
  }

  private void bindRumbles()
  {
    io_copilotLeft.addRumbleTrigger("coral held", new Trigger(s_Coral::getSensor));
    io_copilotRight.addRumbleTrigger("ready to score" , new Trigger(() -> FieldUtils.atReefLineUp(getSwerveState().Pose.getTranslation())));
  }

  public Command getAutonomousCommand() 
  {
    return AutoFactories.getCommandList(SD.IO_AUTO.get(), s_Coral, s_Swerve, this::getSwerveState);
  }
}
