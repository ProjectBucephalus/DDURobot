package frc.robot.constants;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.FieldUtils;

public class FieldConstants 
{
  public static final ArrayList<Translation2d> blueReefMidpoints = FieldUtils.GeoFencing.reefBlue.getMidPoints();
  public static final ArrayList<Translation2d> redReefMidpoints = FieldUtils.GeoFencing.reefRed.getMidPoints();
  
  private static final double reefFaceOffset = 0.4;
  private static final Translation2d reefSidewaysOffset = new Translation2d(0, 0.2);

  private static final Translation2d reefCentre = new Translation2d(4.489, FieldUtils.fieldWidth / 2);
  private static final Translation2d r1CentreLineup = reefCentre.minus(new Translation2d(reefFaceOffset, 0));

  public static final Pose2d raLineup = new Pose2d(r1CentreLineup.plus(reefSidewaysOffset), Rotation2d.kZero);
  public static final Pose2d rbLineup = new Pose2d(r1CentreLineup.minus(reefSidewaysOffset), Rotation2d.kZero);
  public static final Pose2d rcLineup = raLineup.rotateAround(reefCentre, new Rotation2d(Units.degreesToRadians(-60)));
  public static final Pose2d rdLineup = rbLineup.rotateAround(reefCentre, new Rotation2d(Units.degreesToRadians(-60)));
  public static final Pose2d reLineup = raLineup.rotateAround(reefCentre, new Rotation2d(Units.degreesToRadians(-120)));
  public static final Pose2d rfLineup = rbLineup.rotateAround(reefCentre, new Rotation2d(Units.degreesToRadians(-120)));
  public static final Pose2d rgLineup = raLineup.rotateAround(reefCentre, Rotation2d.k180deg);
  public static final Pose2d rhLineup = rbLineup.rotateAround(reefCentre, Rotation2d.k180deg);
  public static final Pose2d riLineup = raLineup.rotateAround(reefCentre, new Rotation2d(Units.degreesToRadians(120)));
  public static final Pose2d rjLineup = rbLineup.rotateAround(reefCentre, new Rotation2d(Units.degreesToRadians(120)));
  public static final Pose2d rkLineup = raLineup.rotateAround(reefCentre, new Rotation2d(Units.degreesToRadians(60)));
  public static final Pose2d rlLineup = rbLineup.rotateAround(reefCentre, new Rotation2d(Units.degreesToRadians(60)));
  public static final Pose2d[] reefLineups = {raLineup, rbLineup, rcLineup, reLineup, rfLineup, rgLineup, rhLineup, riLineup, rjLineup, rkLineup, rlLineup};

  public static Pose2d getLineup(String name)
  {
    switch (name) {
      case "ra":
        return raLineup;
      
      case "rb":
        return rbLineup;
      
      case "rc":
        return rcLineup;

      case "rd":
        return rdLineup;

      case "re":
        return reLineup;

      case "rf":
        return rfLineup;

      case "rg":
        return rgLineup;

      case "rh":
        return rhLineup;

      case "ri":
        return riLineup;

      case "rj":
        return rjLineup;

      case "rk":
        return rkLineup;

      case "rl":
        return rlLineup;
    
      default:
        return raLineup;
    }
  }

  public static final double coralStationRange = 0.6;
}
