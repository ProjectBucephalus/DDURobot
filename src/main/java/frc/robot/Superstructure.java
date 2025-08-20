package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import frc.robot.commands.swerve.*;
import frc.robot.constants.*;
import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.FieldConstants.GeoFencing.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.vision.Vision.TagPOI;
import frc.robot.util.AutoFactories;
import frc.robot.util.FieldUtils;
import frc.robot.util.SD;
import frc.robot.util.controlTransmutation.*;
import frc.robot.util.libs.Telemetry;

public class Superstructure 
{
  public enum TargetPosition {Left, Right, Centre, None}
  public enum DriveState {Reef, Station, Barge, None}
  
  /* State */
  private SwerveDriveState swerveState;
  private TargetPosition currentTarget;
  private DriveState currentDriveState;

  /* Telemetry and SD */
  private Field2d field = new Field2d();
  private final Telemetry ctreLogger = new Telemetry(Constants.Swerve.maxSpeed);
  
  /* Subsystems */
  private final CommandSwerveDrivetrain s_Swerve = TunerConstants.createDrivetrain();
  private final CoralRoller s_Coral = new CoralRoller();
  private final Vision s_Vision = new Vision
    (
      (poseEst, timestmp, stdDevs) -> 
      {
        s_Swerve.setVisionMeasurementStdDevs(stdDevs); 
        s_Swerve.addVisionMeasurement(poseEst, timestmp);
      },
      () -> Pair.of(s_Swerve.getPigeon2().getYaw().getValueAsDouble(), swerveState.Speeds.omegaRadiansPerSecond), 
      new Limelight("limelight-fore"), 
      new Limelight("limelight-aft")
    );

  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  /* Rumble */
  private final RumbleRequester io_driverRight   = new RumbleRequester(driver, RumbleType.kRightRumble, SD.RUMBLE_DRIVER);
  private final RumbleRequester io_driverLeft    = new RumbleRequester(driver, RumbleType.kLeftRumble, SD.RUMBLE_DRIVER);
  private final RumbleRequester io_operatorRight  = new RumbleRequester(operator, RumbleType.kRightRumble, SD.RUMBLE_OPERATOR);
  private final RumbleRequester io_operatorLeft   = new RumbleRequester(operator, RumbleType.kLeftRumble, SD.RUMBLE_OPERATOR);
  
  /* Input Transmutation */
  private final JoystickTransmuter driverStick = new JoystickTransmuter(driver::getLeftY, driver::getLeftX).invertX().invertY();
  private final Brake driverBrake = new Brake(driver::getRightTriggerAxis, Constants.Control.maxThrottle, Constants.Control.minThrottle);
  private final InputCurve driverInputCurve = new InputCurve(2);
  private final Deadband driverDeadband = new Deadband();

  public Superstructure()
  {
    /* State Initialisation */
    currentDriveState = DriveState.None;
    updateSwerveState();
    
    /* Telemetry and SD */
    SmartDashboard.putData("Field", field);
    s_Swerve.registerTelemetry(ctreLogger::telemeterize);
    
    /* Configure Input Transmutation */
    boolean redAlliance = FieldUtils.isRedAlliance();
    driverStick.rotated(redAlliance);
    driverStick.withFieldObjects(GeoFencing.fieldGeoFence).withBrake(driverBrake).withInputCurve(driverInputCurve).withDeadband(driverDeadband);
    FieldUtils.activateAllianceFencing(redAlliance);
    FieldConstants.GeoFencing.configureAttractors((testTarget, testState) -> currentTarget == testTarget && currentDriveState == testState);
    FieldObject.setRobotRadiusSup
      (() -> 
        Math.hypot(swerveState.Speeds.vxMetersPerSecond, swerveState.Speeds.vyMetersPerSecond) >= robotSpeedThreshold ? 
        robotRadiusCircumscribed : 
        robotRadiusInscribed
      );
    FieldObject.setRobotPosSup(swerveState.Pose::getTranslation);

    /* Bindings */
    bindControls();
    bindRumbles();
    bindSD();
  }

  private void updateSwerveState()
  {
    swerveState = s_Swerve.getState();
    field.setRobotPose(swerveState.Pose);
  }

  private void bindControls()
  {
    /* Default Commands */
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
    s_Coral.setDefaultCommand(s_Coral.setSpeedCommand(0));

    /* Setting Drive States */
    driver.povLeft().onTrue(Commands.runOnce(() -> currentTarget = TargetPosition.Left));
    driver.povRight().onTrue(Commands.runOnce(() -> currentTarget = TargetPosition.Right));
    driver.povUp().onTrue(Commands.runOnce(() -> currentTarget = TargetPosition.Centre));
    driver.povDown().onTrue(Commands.runOnce(() -> currentTarget = TargetPosition.None));
    
    driver.x().onTrue(Commands.runOnce(() -> currentDriveState = DriveState.Reef));
    driver.a().onTrue(Commands.runOnce(() -> currentDriveState = DriveState.Station));
    driver.y().onTrue(Commands.runOnce(() -> currentDriveState = DriveState.Barge));
    driver.b().onTrue(Commands.runOnce(() -> currentDriveState = DriveState.None));
    driver.axisMagnitudeGreaterThan(Axis.kRightX.value, 0.2).onTrue(Commands.runOnce(() -> currentDriveState = DriveState.None));
    
    /* Coral Roller */
    driver.leftTrigger().whileTrue(s_Coral.setSpeedCommand(Constants.Coral.forwardSpeed));
    driver.leftBumper().whileTrue(s_Coral.setSpeedCommand(Constants.Coral.reverseSpeed));
    new Trigger(() -> FieldUtils.atReefLineUp(swerveState.Pose)).whileTrue(s_Coral.setSpeedCommand(Constants.Coral.forwardSpeed));

    /* Heading Locking */
    new Trigger(() -> currentDriveState == DriveState.None)
      .onTrue(Commands.runOnce(() -> s_Vision.setActivePOI(TagPOI.REEF)))
      .whileTrue
      (
        new ManualDrive
        (
          s_Swerve, 
          driverStick::stickOutput,
          () -> -driver.getRightX(),
          driver::getRightTriggerAxis
        )
      );
    new Trigger(() -> currentDriveState == DriveState.Reef)
      .onTrue(Commands.runOnce(() -> s_Vision.setActivePOI(TagPOI.REEF)))
      .whileTrue
      (
        new TargetScoreDrive
        (
          s_Swerve, 
          driverStick::stickOutput,
          Rotation2d.kZero,
          swerveState.Pose::getTranslation
        )
      );
    new Trigger(() -> currentDriveState == DriveState.Station)
    .onTrue(Commands.runOnce(() -> s_Vision.setActivePOI(TagPOI.CORALSTATION)))
      .whileTrue
      (
        new TargetStationDrive
        (
          s_Swerve, 
          driverStick::stickOutput,
          Rotation2d.kZero,
          swerveState.Pose::getTranslation
        )
      );
    new Trigger(() -> currentDriveState == DriveState.Barge)
      .onTrue(Commands.runOnce(() -> s_Vision.setActivePOI(TagPOI.BARGE)))
      .whileTrue
      (
        new HeadingLockedDrive
        (
          s_Swerve, 
          driverStick::stickOutput,
          Rotation2d.kZero,
          Rotation2d.kZero,
          swerveState.Pose::getTranslation
        )
      );
    
    /* Other */
    driver.start().onTrue(Commands.runOnce(s_Vision::resetRotation).ignoringDisable(true));
  }

  private void bindRumbles()
  {
    io_operatorLeft.addRumbleTrigger("CoralHeld", new Trigger(s_Coral::getSensor));
    io_operatorRight.addRumbleTrigger("ScoreReady" , new Trigger(() -> FieldUtils.atReefLineUp(swerveState.Pose)));
  }

  private void bindSD()
  {
    new Trigger(SD.LL_EXPOSURE_UP::button).onTrue(Commands.runOnce(s_Vision::incrementPipeline));
    new Trigger(SD.LL_EXPOSURE_DOWN::button).onTrue(Commands.runOnce(s_Vision::decrementPipeline));
  }

  public void periodic()
  {
    updateSwerveState();
  }

  public Command getAutonomousCommand() 
  {
    return AutoFactories.getCommandList(SD.AUTO_STRING.get(), s_Coral, s_Swerve, () -> swerveState);
  }

  public void setYaw(double newYaw) {s_Swerve.getPigeon2().setYaw(newYaw);}
}
