// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Superstructure;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers;

public class Limelight  
{  
  private LimelightHelpers.PoseEstimate mt2;
  private LimelightHelpers.PoseEstimate mt1;
  
  private final String limelightName;

  public static boolean rotationKnown = false;
  private ArrayList<Double> rotationData = new ArrayList<Double>();
  private boolean lastCycleRotationKnown = false;
  private final int mt1CyclesNeeded = 10;

  public enum TagPOI 
  {
    REEF,
    BARGE,
    PROCESSOR,
    CORALSTATION
  }
  
  /** Creates a new Limelight. */
  public Limelight(String name) 
  {
    limelightName = name;
  }

  public void setIMUMode(int mode)
    {LimelightHelpers.SetIMUMode(limelightName, mode);}

  public Rotation2d getLimelightRotation()
  {
    mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

    if (mt1 != null && mt1.avgTagDist < 4)
      {return mt1.pose.getRotation();}
    return Rotation2d.kZero;
  }

  protected void updateValidIDs(int[] validIDs)
  {
    LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, validIDs);
  }

  protected void updatePipeline(int pipelineIndex)
  {
    LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);
  }

  @Logged
  public Pose3d getMT1Pose()
    {return LimelightHelpers.getBotPose3d_wpiBlue(limelightName);}

  public PoseEstimate getMT2(double headingDeg)
  {
    LimelightHelpers.SetRobotOrientation(limelightName, headingDeg, 0, 0, 0, 0, 0);
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
  }

  public void periodic() 
  { 
    if (Superstructure.isVisionActive())
    {
      rotationKnown = Superstructure.isRotationKnown();

      if (!rotationKnown) 
      {
        lastCycleRotationKnown = false;
        if (!getLimelightRotation().equals(Rotation2d.kZero))
        {
          rotationData.add(0, getLimelightRotation().getDegrees());
    
          if (rotationData.size() > mt1CyclesNeeded)
            {rotationData.remove(mt1CyclesNeeded);}
    
          if (rotationData.size() == mt1CyclesNeeded)
          {
            double lowest = rotationData.get(0).doubleValue();
            double highest = rotationData.get(0).doubleValue();
            
            for(int i = 1; i < mt1CyclesNeeded; i++)
            {
              lowest = Math.min(lowest, rotationData.get(i).doubleValue());
              highest = Math.max(highest, rotationData.get(i).doubleValue());
            }
            
            if (highest - lowest < 1)
            {
              rotationKnown = true;
              Superstructure.setRotationKnown(true);
              //SmartDashboard.putNumber("limelight " + limelightName + " average rotation reading", (highest + lowest) / 2);
              Superstructure.setYaw((highest + lowest) / 2);
            }
          }
        }
      }

      if (!lastCycleRotationKnown) 
      {
        if (rotationKnown) 
        {
          rotationData.clear();
          lastCycleRotationKnown = true;
          //RobotContainer.s_Swerve.resetPose(new Pose2d(RobotContainer.swerveState.Pose.getTranslation(), new Rotation2d(Math.toRadians(RobotContainer.s_Swerve.getPigeon2().getYaw().getValueAsDouble()))));
        }
      }

      //omegaRps = Units.radiansToRotations(RobotContainer.swerveState.Speeds.omegaRadiansPerSecond);
    }
  }
}
