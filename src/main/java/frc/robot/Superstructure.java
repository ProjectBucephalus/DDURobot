package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import frc.robot.commands.swerve.*;
import frc.robot.constants.*;
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
  
  private static boolean rotationKnown = false;
  private static boolean useLimelights = true;
  private static boolean useFence = true;
  private boolean redAlliance;

  private SwerveDriveState swerveState;
  private Field2d field;
  
  private final Telemetry ctreLogger;
  
  private final CommandSwerveDrivetrain s_Swerve;
  private final CoralRoller s_Coral;
  private final Vision s_Vision;
  
  private TargetPosition currentTarget;
  private DriveState currentDriveState;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final RumbleRequester io_driverRight   = new RumbleRequester(driver, RumbleType.kRightRumble, SD.RUMBLE_DRIVER);
  private final RumbleRequester io_driverLeft    = new RumbleRequester(driver, RumbleType.kLeftRumble, SD.RUMBLE_DRIVER);
  private final RumbleRequester io_operatorRight  = new RumbleRequester(operator, RumbleType.kRightRumble, SD.RUMBLE_OPERATOR);
  private final RumbleRequester io_operatorLeft   = new RumbleRequester(operator, RumbleType.kLeftRumble, SD.RUMBLE_OPERATOR);
  
  private final JoystickTransmuter driverStick = new JoystickTransmuter(driver::getLeftY, driver::getLeftX).invertX().invertY();
  private final Brake driverBrake = new Brake(driver::getRightTriggerAxis, Constants.Control.maxThrottle, Constants.Control.minThrottle);
  private final InputCurve driverInputCurve = new InputCurve(2);
  private final Deadband driverDeadband = new Deadband();

  public Superstructure()
  {
    field = new Field2d();
    s_Swerve = TunerConstants.createDrivetrain();
    s_Coral = new CoralRoller();
    s_Vision = new Vision(this::applyVisionEstimate, () -> Pair.of(getYaw(), swerveState.Speeds.omegaRadiansPerSecond), new Limelight("limelight-fore"), new Limelight("limelight-aft"));

    ctreLogger = new Telemetry(Constants.Swerve.maxSpeed);

    redAlliance = FieldUtils.isRedAlliance();

    setStartPose(redAlliance);
    updateSwerveState();
    driverStick.rotated(redAlliance);
    FieldUtils.activateAllianceFencing();

    SmartDashboard.putData("Field", field);

    s_Swerve.registerTelemetry(ctreLogger::telemeterize);
    driverStick.withFieldObjects(FieldConstants.GeoFencing.fieldGeoFence).withBrake(driverBrake).withInputCurve(driverInputCurve).withDeadband(driverDeadband);
    FieldConstants.GeoFencing.fieldGeoFence.setActiveCondition(() -> useFence && useLimelights);
    FieldConstants.GeoFencing.configureAttractors((testTarget, testState) -> currentTarget == testTarget && currentDriveState == testState);
    FieldObject.setRobotRadiusSup(this::robotRadiusSup);
    FieldObject.setRobotPosSup(this::getPosition);

    bindControls();
    bindRumbles();
    bindSD();
  }

  public SwerveDriveState getSwerveState() 
    {return swerveState;}

  public void updateSwerveState()
  {
    swerveState = s_Swerve.getState();
    field.setRobotPose(swerveState.Pose);
  }

  public Translation2d getPosition()
  {
    return swerveState.Pose.getTranslation();
  }

  private void bindControls()
  {
    currentDriveState = DriveState.None;
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
          () -> getPosition()
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
          () -> getPosition()
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
          () -> getPosition()
        )
      );
    

    driver.leftTrigger().whileTrue(s_Coral.setSpeedCommand(Constants.Coral.forwardSpeed));
    driver.leftBumper().whileTrue(s_Coral.setSpeedCommand(Constants.Coral.reverseSpeed));
    //operator.leftBumper().whileTrue(s_Coral.runCommand(SD.IO_CORALSPEED_R.get()));
    // Heading reset
    driver.start()
      .onTrue
      (
        Commands.runOnce
        (
          () -> 
          {
            Pigeon2 pigeon = s_Swerve.getPigeon2();

            pigeon.setYaw(redAlliance ? 0 : 180);
            s_Swerve.resetPose(new Pose2d(swerveState.Pose.getTranslation(), new Rotation2d(Math.toRadians(pigeon.getYaw().getValueAsDouble()))));
            FieldUtils.activateAllianceFencing();
            rotationKnown = false;
            useLimelights = true;
          }
        )
        .ignoringDisable(true)
        .withName("HeadingReset")
      );
    driver.back()
      .onTrue
      (
        Commands.runOnce
        (
          () -> 
          {
            Pigeon2 pigeon = s_Swerve.getPigeon2();

            pigeon.setYaw(redAlliance ? 0 : 180);
            s_Swerve.resetPose(new Pose2d(swerveState.Pose.getTranslation(), new Rotation2d(Math.toRadians(pigeon.getYaw().getValueAsDouble()))));
            FieldUtils.activateAllianceFencing();
            rotationKnown = false;
            useLimelights = false;
          }
        )
        .ignoringDisable(true)
        .withName("DisableNavigation")
      );
    
    driver.povLeft().onTrue(Commands.runOnce(() -> currentTarget = TargetPosition.Left));
    driver.povRight().onTrue(Commands.runOnce(() -> currentTarget = TargetPosition.Right));
    driver.povUp().onTrue(Commands.runOnce(() -> currentTarget = TargetPosition.Centre));
    driver.povDown().onTrue(Commands.runOnce(() -> currentTarget = TargetPosition.None));

    driver.x().onTrue(Commands.runOnce(() -> currentDriveState = DriveState.Reef));
    driver.a().onTrue(Commands.runOnce(() -> currentDriveState = DriveState.Station));
    driver.y().onTrue(Commands.runOnce(() -> currentDriveState = DriveState.Barge));
    driver.b().onTrue(Commands.runOnce(() -> currentDriveState = DriveState.None));
    driver.axisMagnitudeGreaterThan(Axis.kRightX.value, 0.2).onTrue(Commands.runOnce(() -> currentDriveState = DriveState.None));
    
    new Trigger(() -> FieldUtils.atReefLineUp(swerveState.Pose.getTranslation())).whileTrue(s_Coral.setSpeedCommand(Constants.Coral.forwardSpeed));
  }

  private void bindRumbles()
  {
    io_operatorLeft.addRumbleTrigger("CoralHeld", new Trigger(s_Coral::getSensor));
    io_operatorRight.addRumbleTrigger("ScoreReady" , new Trigger(() -> FieldUtils.atReefLineUp(swerveState.Pose.getTranslation())));
  }

  private void bindSD()
  {
    new Trigger(SD.LL_EXPOSURE_UP::button).onTrue(Commands.runOnce(s_Vision::incrementPipeline));
    new Trigger(SD.LL_EXPOSURE_DOWN::button).onTrue(Commands.runOnce(s_Vision::decrementPipeline));
  }

  private void applyVisionEstimate(Pose2d poseEstMeters, double timestampSeconds, Matrix<N3, N1> stdDevs)
  {
    s_Swerve.setVisionMeasurementStdDevs(stdDevs);
    s_Swerve.addVisionMeasurement(poseEstMeters, timestampSeconds);
  }

  public void periodic()
  {
    updateSwerveState();
  }

  public Command getAutonomousCommand() 
  {
    return AutoFactories.getCommandList(SD.AUTO_STRING.get(), s_Coral, s_Swerve, () -> swerveState);
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

  public static boolean isRotationKnown() {return rotationKnown;}
  public static void    setRotationKnown(boolean isKnown) {rotationKnown = isKnown;}
  public static boolean isVisionActive() {return useLimelights;}
  public void    setYaw(double newYaw) {s_Swerve.getPigeon2().setYaw(newYaw);}
  public double getYaw() {return s_Swerve.getPigeon2().getYaw().getValueAsDouble();}
}
