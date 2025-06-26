package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.ManualDrive;
import frc.robot.constants.Constants;
import frc.robot.constants.IDConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RumbleRequester;
import frc.robot.util.AutoFactories;
import frc.robot.util.FieldUtils;
import frc.robot.util.SD;
import frc.robot.util.Telemetry;
import frc.robot.util.controlTransmutation.Brake;
import frc.robot.util.controlTransmutation.FieldObject;
import frc.robot.util.controlTransmutation.JoystickTransmuter;

public class Superstructure 
{
  enum TargetPosition {Left, Right, Centre}
  
  private final Telemetry logger;
  
  private SwerveDriveState swerveState;
  private Field2d field;
  
  private static CommandSwerveDrivetrain s_Swerve;
  private static CoralRoller s_Coral;
  private static Limelight s_foreLL;
  private static Limelight s_aftLL;
  
  private TargetPosition currentTarget;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController copilot = new CommandXboxController(1);
  private final RumbleRequester io_driverRight   = new RumbleRequester(driver, RumbleType.kRightRumble, SD.RUMBLE_D_R::put, SD.IO_RUMBLE_D::get);
  private final RumbleRequester io_driverLeft    = new RumbleRequester(driver, RumbleType.kLeftRumble, SD.RUMBLE_D_L::put, SD.IO_RUMBLE_D::get);
  private final RumbleRequester io_copilotRight  = new RumbleRequester(copilot, RumbleType.kRightRumble, SD.RUMBLE_C_R::put, SD.IO_RUMBLE_C::get);
  private final RumbleRequester io_copilotLeft   = new RumbleRequester(copilot, RumbleType.kLeftRumble, SD.RUMBLE_C_L::put, SD.IO_RUMBLE_C::get);
  
  private final JoystickTransmuter driverStick = new JoystickTransmuter(driver::getLeftY, driver::getLeftX);
  private final Brake driverBrake = new Brake(driver::getRightTriggerAxis, Constants.Control.maxThrottle, Constants.Control.minThrottle);

  public Superstructure()
  {
    field = new Field2d();
    s_Swerve = TunerConstants.createDrivetrain();
    s_Coral = new CoralRoller();
    s_foreLL = new Limelight("fore");
    s_aftLL = new Limelight("aft");

    logger = new Telemetry(Constants.Swerve.maxSpeed);

    setStartPose(FieldUtils.isRedAlliance());
    updateSwerveState();

    SmartDashboard.putData("Field", field);

    s_Swerve.registerTelemetry(logger::telemeterize);
    driverStick.withFieldObjects(FieldConstants.GeoFencing.fieldGeoFence).withBrake(driverBrake);
    FieldConstants.GeoFencing.fieldGeoFence.setInactive();
    FieldObject.setRobotRadiusSup(this::robotRadiusSup);
    FieldObject.setRobotPosSup(swerveState.Pose::getTranslation);

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
        driverStick::stickOutput,
        () -> -driver.getRightX(),
        driver::getRightTriggerAxis
      )
    );

    copilot.leftTrigger().whileTrue(s_Coral.smartRunCommand(SD.IO_CORALSPEED_F.get()));
    copilot.rightTrigger().whileTrue(s_Coral.smartRunCommand(SD.IO_CORALSPEED_R.get()));
    copilot.leftBumper().whileTrue(s_Coral.runCommand(SD.IO_CORALSPEED_F.get()));
    copilot.rightBumper().whileTrue(s_Coral.runCommand(SD.IO_CORALSPEED_R.get()));
    new Trigger(() -> FieldUtils.atReefLineUp(getSwerveState().Pose.getTranslation())).whileTrue(s_Coral.smartRunCommand(Constants.Coral.forwardSpeed));
    
    copilot.povLeft().onTrue(Commands.runOnce(() -> currentTarget = TargetPosition.Left));
    copilot.povRight().onTrue(Commands.runOnce(() -> currentTarget = TargetPosition.Right));
    copilot.povUp().onTrue(Commands.runOnce(() -> currentTarget = TargetPosition.Centre));
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

  public static void addVisionMeasurement(Pose2d poseMeasurement, double timestamp)
  {
    s_Swerve.addVisionMeasurement(poseMeasurement, timestamp);
  }

  public static void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs)
  {
    s_Swerve.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
  }
  
  public double robotRadiusSup() 
  {
    double robotSpeed = Math.hypot(swerveState.Speeds.vxMetersPerSecond, swerveState.Speeds.vyMetersPerSecond);

    return
    robotSpeed >= FieldConstants.GeoFencing.robotSpeedThreshold ?
    FieldConstants.GeoFencing.robotRadiusCircumscribed :
    FieldConstants.GeoFencing.robotRadiusInscribed;
  }

  private void setStartPose(boolean isRedAlliance)
  {
    if (isRedAlliance)
      {s_Swerve.resetPose(FieldConstants.redStartLine);}
    else
      {s_Swerve.resetPose(FieldConstants.blueStartLine);}
  }
}
