package frc.robot;

import java.util.Set;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
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
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.HeadingLockedDrive;
import frc.robot.commands.swerve.ManualDrive;
import frc.robot.commands.swerve.TargetCageDrive;
import frc.robot.commands.swerve.TargetScoreDrive;
import frc.robot.commands.swerve.TargetStationDrive;
import frc.robot.constants.Constants;
import frc.robot.constants.IDConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RumbleRequester;
import frc.robot.subsystems.Limelight.TagPOI;
import frc.robot.util.AutoFactories;
import frc.robot.util.FieldUtils;
import frc.robot.util.SD;
import frc.robot.util.Telemetry;
import frc.robot.util.controlTransmutation.Brake;
import frc.robot.util.controlTransmutation.Deadband;
import frc.robot.util.controlTransmutation.FieldObject;
import frc.robot.util.controlTransmutation.InputCurve;
import frc.robot.util.controlTransmutation.JoystickTransmuter;

public class Superstructure 
{
  public enum TargetPosition {Left, Right, Centre, None}
  public enum DriveState {Reef, Station, Barge, None}
  
  private static boolean rotationKnown = false;
  private static boolean useLimelights = true;
  private static boolean useFence = true;
  private static boolean useRestrictors = true;
  private static boolean redAlliance;
  
  private final Telemetry logger;
  
  private SwerveDriveState swerveState;
  private Field2d field;
  
  private static CommandSwerveDrivetrain s_Swerve;
  private static CoralRoller s_Coral;
  private static Limelight s_foreLL;
  private static Limelight s_aftLL;
  
  private static TargetPosition currentTarget;
  private static DriveState currentDriveState;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final RumbleRequester io_driverRight   = new RumbleRequester(driver, RumbleType.kRightRumble, SD.RUMBLE_D_R::put, SD.IO_RUMBLE_D::get);
  private final RumbleRequester io_driverLeft    = new RumbleRequester(driver, RumbleType.kLeftRumble, SD.RUMBLE_D_L::put, SD.IO_RUMBLE_D::get);
  private final RumbleRequester io_copilotRight  = new RumbleRequester(operator, RumbleType.kRightRumble, SD.RUMBLE_C_R::put, SD.IO_RUMBLE_C::get);
  private final RumbleRequester io_copilotLeft   = new RumbleRequester(operator, RumbleType.kLeftRumble, SD.RUMBLE_C_L::put, SD.IO_RUMBLE_C::get);
  
  private final JoystickTransmuter driverStick = new JoystickTransmuter(driver::getLeftY, driver::getLeftX).invertX().invertY();
  private final Brake driverBrake = new Brake(driver::getRightTriggerAxis, Constants.Control.maxThrottle, Constants.Control.minThrottle);
  private final InputCurve driverInputCurve = new InputCurve(2);
  private final Deadband driverDeadband = new Deadband();

  public Superstructure()
  {
    field = new Field2d();
    s_Swerve = TunerConstants.createDrivetrain();
    s_Coral = new CoralRoller();
    s_foreLL = new Limelight("limelight-fore");
    s_aftLL = new Limelight("limelight-aft");

    logger = new Telemetry(Constants.Swerve.maxSpeed);

    setStartPose(getAlliance());
    updateSwerveState();
    driverStick.rotated(redAlliance);
    FieldUtils.activateAllianceFencing();

    SmartDashboard.putData("Field", field);

    s_Swerve.registerTelemetry(logger::telemeterize);
    driverStick.withFieldObjects(FieldConstants.GeoFencing.fieldGeoFence).withBrake(driverBrake).withInputCurve(driverInputCurve).withDeadband(driverDeadband);
    FieldConstants.GeoFencing.fieldGeoFence.setActiveCondition(() -> isFenceActive() && isVisionActive());
    FieldObject.setRobotRadiusSup(this::robotRadiusSup);
    FieldObject.setRobotPosSup(() -> getPosition());

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

    new Trigger(() -> {return currentDriveState == DriveState.None;})
      .onTrue(Commands.runOnce(() -> Limelight.setActivePOI(TagPOI.REEF)))
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

    new Trigger(() -> {return currentDriveState == DriveState.Reef;})
      .onTrue(Commands.runOnce(() -> Limelight.setActivePOI(TagPOI.REEF)))
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

    new Trigger(() -> {return currentDriveState == DriveState.Station;})
    .onTrue(Commands.runOnce(() -> Limelight.setActivePOI(TagPOI.CORALSTATION)))
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
    
    new Trigger(() -> {return currentDriveState == DriveState.Barge;})
      .onTrue(Commands.runOnce(() -> Limelight.setActivePOI(TagPOI.BARGE)))
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
    

    driver.leftTrigger().whileTrue(s_Coral.runCommand(SD.IO_CORALSPEED_F.get()));
    driver.leftBumper().whileTrue(s_Coral.runCommand(SD.IO_CORALSPEED_R.get()));
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

            pigeon.setYaw(getAlliance() ? 0 : 180);
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

            pigeon.setYaw(getAlliance() ? 0 : 180);
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

  public static boolean isRotationKnown() {return rotationKnown;}
  public static void    setRotationKnown(boolean isKnown) {rotationKnown = isKnown;}
  public static boolean isVisionActive() {return useLimelights;}
  public static boolean isFenceActive() {return useFence;}
  public static boolean isRestrictorsActive() {return useRestrictors;}
  public static boolean isRedAlliance() {return redAlliance;}
  public static boolean getAlliance() {return redAlliance = FieldUtils.isRedAlliance();}
  public static void    setYaw(double newYaw) {s_Swerve.getPigeon2().setYaw(newYaw);}
  public static double  getYaw() {return s_Swerve.getPigeon2().getYaw().getValueAsDouble();}
  public static boolean checkTargetPosition(TargetPosition testTargetPosition) {return testTargetPosition == currentTarget;}
  public static boolean checkDriveState(DriveState testDriveState) {return testDriveState == currentDriveState;}
}
