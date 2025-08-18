// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
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
import frc.robot.util.SD;
import frc.robot.util.libs.LimelightHelpers;

public class Vision extends SubsystemBase 
{
  private final PoseEstimateConsumer consumer;
  private final Supplier<Pair<Double, Double>> rotationData;
  private final Limelight[] lls;

  private int[] validIDs = Constants.Vision.reefIDs;
  private int pipelineIndex = (int)SD.LL_EXPOSURE.defaultValue();

  private ArrayList<Double> rotationBuf = new ArrayList<Double>();
  private boolean lastCycleRotationKnown = false;
  private final int mt1CyclesNeeded = 10;

  /** Creates a new Vision. */
  public Vision(PoseEstimateConsumer consumer, Supplier<Pair<Double, Double>> rotationData, Limelight... lls) 
  {
    this.consumer = consumer;
    this.rotationData = rotationData;
    this.lls = lls;
  }

  public void setActivePOI(TagPOI activePOI) 
  {
    validIDs =
    switch (activePOI) 
    {
      case REEF -> Constants.Vision.reefIDs;
      case BARGE -> Constants.Vision.bargeIDs;
      case CORALSTATION, PROCESSOR -> Constants.Vision.humanPlayerStationIDs;
      default -> Constants.Vision.reefIDs;
    };
  }

  public void incrementPipeline() 
  {
    pipelineIndex = MathUtil.clamp(pipelineIndex + 1, 0, 7);
    for (var ll : lls) {ll.updatePipeline(pipelineIndex);}
    SD.LL_EXPOSURE.put(pipelineIndex);
  }

  public void decrementPipeline()
  {
    pipelineIndex = MathUtil.clamp(pipelineIndex - 1, 0, 7);
    for (var ll : lls) {ll.updatePipeline(pipelineIndex);}
    SD.LL_EXPOSURE.put(pipelineIndex);
  }

  @Override
  public void periodic() 
  {
    for (var ll : lls)
    {    
      ll.updateValidIDs(validIDs);
    } 

    if (SD.LL_TOGGLE.get()) 
    {
      for (var ll : lls)
      {
        var rotationVals = rotationData.get();
        double heading = rotationVals.getFirst();
        double omegaRps = rotationVals.getSecond();

        var mt2 = ll.getMT2(heading);
        //mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        
        boolean useUpdate = !(mt2 == null || mt2.tagCount == 0 || omegaRps > 2.0);
        
        if (useUpdate) 
        {
          double stdDevFactor = Math.pow(mt2.avgTagDist, 2.0) / mt2.tagCount;

          double linearStdDev = Constants.Vision.linearStdDevBaseline * stdDevFactor;
          double rotStdDev = Constants.Vision.rotStdDevBaseline * stdDevFactor;

          consumer.accept(mt2.pose, Utils.fpgaToCurrentTime(mt2.timestampSeconds), VecBuilder.fill(linearStdDev, linearStdDev, rotStdDev));
        }
      }
    }

    if (Superstructure.isVisionActive())
    {
      boolean rotationKnown = Superstructure.isRotationKnown();

      if (!rotationKnown) 
      {
        lastCycleRotationKnown = false;
        if (!getLimelightRotation().equals(Rotation2d.kZero))
        {
          rotationBuf.add(0, getLimelightRotation().getDegrees());
    
          if (rotationBuf.size() > mt1CyclesNeeded)
            {rotationBuf.remove(mt1CyclesNeeded);}
    
          if (rotationBuf.size() == mt1CyclesNeeded)
          {
            double lowest = rotationBuf.get(0).doubleValue();
            double highest = rotationBuf.get(0).doubleValue();
            
            for(int i = 1; i < mt1CyclesNeeded; i++)
            {
              lowest = Math.min(lowest, rotationBuf.get(i).doubleValue());
              highest = Math.max(highest, rotationBuf.get(i).doubleValue());
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
          rotationBuf.clear();
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
