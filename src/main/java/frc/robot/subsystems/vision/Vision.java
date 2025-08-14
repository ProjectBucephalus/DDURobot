// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Superstructure;
import frc.robot.constants.Constants;
import frc.robot.subsystems.vision.Limelight.TagPOI;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.SD;

public class Vision extends SubsystemBase 
{
  private final PoseEstimateConsumer consumer;
  private final Supplier<Rotation2d> gyro;
  private final Limelight[] lls;

  private final TimeInterpolatableBuffer<Rotation2d> headingBuffer =
    TimeInterpolatableBuffer.createBuffer(1.0);
  private static int[] validIDs = Constants.Vision.reefIDs;

  private int pipelineIndex = (int)SD.IO_LL_EXPOSURE.defaultValue();
  private boolean pipelineUpdated = false;

  /** Creates a new Vision. */
  public Vision(PoseEstimateConsumer consumer, Supplier<Rotation2d> gyro, Limelight... lls) 
  {
    this.consumer = consumer;
    this.gyro = gyro;
    this.lls = lls;
  }

  public void setActivePOI(TagPOI activePOI) 
  {
    switch (activePOI) 
    {
      default:
      case REEF:
        validIDs = Constants.Vision.reefIDs;
        break;
      case BARGE:
        validIDs = Constants.Vision.bargeIDs;
        break;
      case PROCESSOR:
      case CORALSTATION:
        validIDs = Constants.Vision.humanPlayerStationIDs;
        break;
    }
  }

  public int updatePipeline()
  {
    if (SD.IO_LL_EXPOSURE_UP.button())
    {
      SD.IO_LL_EXPOSURE.put(MathUtil.clamp(SD.IO_LL_EXPOSURE.get().intValue() + 1, 0, 7));
    }
    if (SD.IO_LL_EXPOSURE_DOWN.button())
    {
      SD.IO_LL_EXPOSURE.put(MathUtil.clamp(SD.IO_LL_EXPOSURE.get().intValue() - 1, 0, 7));
    }

    return SD.IO_LL_EXPOSURE.get().intValue();
  }

  @Override
  public void periodic() 
  {
    headingBuffer.addSample(RobotController.getTime(), gyro.get());

    int currentPipeline = updatePipeline();
    if (currentPipeline != pipelineIndex)
    {
      pipelineIndex = currentPipeline;
      pipelineUpdated = true;
    }

    for (var ll : lls)
    {    
      ll.updateValidIDs(validIDs);

      if (pipelineUpdated)
      {
        ll.updatePipeline(pipelineIndex);
        pipelineUpdated = false;
      }
    } 

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
    }
  }

  @FunctionalInterface
  public static interface PoseEstimateConsumer 
  {
    public void accept
    (
      Pose2d visionRobotPoseMeters, 
      double timestampSeconds, 
      Matrix<N3, N1> visionMeasurementStdDevs
    );
  }
}
