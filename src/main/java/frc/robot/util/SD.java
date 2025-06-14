// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Set;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Simplified interface for most SmartDashboard interactions */
public class SD 
{
  public static final BooleanKey IO_LL = new BooleanKey("Use Limelight", true);
  public static final DoubleKey  IO_LL_EXPOSURE = new DoubleKey("Exposure Setting", 0);
  public static final BooleanKey IO_LL_EXPOSURE_UP = new BooleanKey("Increase Exposure", false);
  public static final BooleanKey IO_LL_EXPOSURE_DOWN = new BooleanKey("Decrease Exposure", false);

  public static final StringKey  STATE_HEADING = new StringKey("Heading State", "");
  public static final StringKey  STATE_DRIVE = new StringKey("Drive State", "Disabled");
  public static final BooleanKey STATE_HEADING_SNAP = new BooleanKey("Heading Snap Updating", true);

  public static final BooleanKey IO_GEOFENCE = new BooleanKey("Use Fence", true);
  public static final BooleanKey IO_OUTER_GEOFENCE = new BooleanKey("Wall Fence", true);
  public static final DoubleKey  IO_GEOFENCE_IMPACT = new DoubleKey("Fence Impact", 1);
  public static final DoubleKey  IO_RUMBLE_D = new DoubleKey("Driver Rumble", Constants.RumblerConstants.driverDefault);
  public static final DoubleKey  IO_RUMBLE_C = new DoubleKey("Copilot Rumble", Constants.RumblerConstants.copilotDefault);

  public static final DoubleKey SENSOR_GYRO = new DoubleKey("Gyro yaw", 0);

  public static final StringKey RUMBLE_D_R = new StringKey("DriverRight Rumble Queue", "");
  public static final StringKey RUMBLE_D_L = new StringKey("DriverLeft Rumble Queue", "");
  public static final StringKey RUMBLE_C_R = new StringKey("CopilotRight Rumble Queue", "");
  public static final StringKey RUMBLE_C_L = new StringKey("CopilotLeft Rumble Queue", "");

  public static final DoubleKey IO_POSE_X = new DoubleKey("Pose X", 0.0);
  public static final DoubleKey IO_POSE_Y = new DoubleKey("Pose Y", 0.0);
  public static final DoubleKey IO_POSE_R = new DoubleKey("Pose Rotation", 0.0);
  public static final StringKey IO_AUTO = new StringKey("Auto String", "");

  static
  {
    for 
    (
      Initable key : 
      Set.of
      (
        IO_POSE_X,
        IO_POSE_Y,
        IO_POSE_R
      )
    )
    {
      key.init();
    }
  }

  public static void initSwerveDisplay(CommandSwerveDrivetrain s_Swerve)
  {
    SmartDashboard.putData
    (
      "Swerve Drive", 
      new Sendable() 
      {
        @Override
        public void initSendable(SendableBuilder builder) 
        {
          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty("Front Left Angle", () -> s_Swerve.getModule(0).getCurrentState().angle.getRadians(), null);
          builder.addDoubleProperty("Front Left Velocity", () -> s_Swerve.getModule(0).getCurrentState().speedMetersPerSecond, null);

          builder.addDoubleProperty("Front Right Angle", () -> s_Swerve.getModule(1).getCurrentState().angle.getRadians(), null);
          builder.addDoubleProperty("Front Right Velocity", () -> s_Swerve.getModule(1).getCurrentState().speedMetersPerSecond, null);

          builder.addDoubleProperty("Back Left Angle", () -> s_Swerve.getModule(2).getCurrentState().angle.getRadians(), null);
          builder.addDoubleProperty("Back Left Velocity", () -> s_Swerve.getModule(2).getCurrentState().speedMetersPerSecond, null);

          builder.addDoubleProperty("Back Right Angle", () -> s_Swerve.getModule(3).getCurrentState().angle.getRadians(), null);
          builder.addDoubleProperty("Back Right Velocity", () -> s_Swerve.getModule(3).getCurrentState().speedMetersPerSecond, null);

          //!TODO FIX THIS
          //DoubleProperty("Robot Angle", () -> RobotContainer.swerveState.Pose.getRotation().getRadians(), null);
        }
      }
    );
  }

  public interface Initable
  {
    public void init();
  }

  public record BooleanKey (String label, boolean defaultValue) implements Initable
  {
    public boolean get() {return SmartDashboard.getBoolean(label, defaultValue);}

    public boolean button() 
    {
      if (SmartDashboard.getBoolean(label, defaultValue)) 
      {
        put(false); 
        return true;
      } 
      else 
        {return false;}
    }

    public void init() {SmartDashboard.putBoolean(label, defaultValue);}

    public void put(boolean value) {SmartDashboard.putBoolean(label, value);}
  }

  public record DoubleKey (String label, double defaultValue) implements Initable
  {
    public Double get() {return SmartDashboard.getNumber(label, defaultValue);}

    public void init() {SmartDashboard.putNumber(label, defaultValue);}

    public void put(double value) {SmartDashboard.putNumber(label, value);}
  }

  public record StringKey (String label, String defaultValue) implements Initable
  {
    public String get() {return SmartDashboard.getString(label, defaultValue);}

    public void init() {SmartDashboard.putString(label, defaultValue);}

    public void put(String value) {SmartDashboard.putString(label, value);}
  }
}
