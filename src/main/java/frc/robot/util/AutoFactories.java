package frc.robot.util;

import java.util.ArrayList;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralRoller;

public class AutoFactories 
{
  private static final PathConstraints defaultConstraints = Constants.Auto.defaultConstraints;
  private static final PathConstraints slowedConstraints = Constants.Auto.slowedConstraints;
  private static final PathConstraints stationConstraints = Constants.Auto.stationConstraints;

  /**
   * Splits a string of auto command phrases and gets the path command and robot command associated with each command phrase
   * @param commandInput The string of commands to split, seperated by commas with no spaces (e.g. "a1,rA1,p,cR3")
   * @return An array of commands, from the input command phrase string, in the same order
   */
  public static Command getCommandList(String commandInput, CoralRoller s_Coral, CommandSwerveDrivetrain s_Swerve, Supplier<SwerveDriveState> swerveStateSup)
  {
    // Removes all space characters from the single-String command phrases, ensures it's all lowercase, and then splits it into individual strings, which are stored in an array
    String[] splitCommands = commandInput.replaceAll("//s", "").toLowerCase().split(",");
    // The arraylist that all the commands will be placed into
    ArrayList<Command> commandList = new ArrayList<>();

    // Holders for the values during each command processing
    AutoMapping autoMapValue;
    PathPlannerPath nextPath;

    // For each command phrase, adds the associated path and then the associated command to the command list
    for (String splitCommand : splitCommands) 
    {
      switch (splitCommand.charAt(0)) 
      {
        case 'g':
          int seperatorIndex = splitCommand.indexOf(":");
          Translation2d posTarget = 
          new Translation2d
          (
            MathUtil.clamp(Double.parseDouble(splitCommand.substring(1, seperatorIndex)), 0.5, (FieldUtils.fieldLength / 2) - 0.5), 
            MathUtil.clamp(Double.parseDouble(splitCommand.substring(seperatorIndex + 1)), 0.5, FieldUtils.fieldWidth - 0.5)
          );

          if (FieldUtils.isRedAlliance()) 
          {
            posTarget = posTarget.rotateAround(FieldUtils.fieldCentre, Rotation2d.k180deg);
          }

          Translation2d finalPosTarget = posTarget;
          commandList.add(Commands.defer(() -> AutoBuilder.pathfindToPose(new Pose2d(finalPosTarget, RobotContainer.swerveState.Pose.getRotation()), defaultConstraints), Set.of()));
          break;

        case 'w':
          commandList.add(Commands.waitSeconds(Double.parseDouble(splitCommand.substring(1))));
          break;

        case 't':
          double targetMatchTimeElapsed = Double.parseDouble(splitCommand.substring(1));

          commandList.add(Commands.waitUntil(() -> Timer.getMatchTime() < (15 - targetMatchTimeElapsed)));
          break;

        case 'r':
          commandList.add
          (
            s_Swerve.poseLockDriveCommand(() -> FieldConstants.getReefLineup(splitCommand), swerveStateSup)
          );

          commandList.add(Commands.waitSeconds(0.2)); //TODO: Test Removal

          commandList.add(s_Coral.setStatusCommand(CoralManipulator.Status.DELIVERY_SMART));
          
          if (splitCommand.charAt(2) == '4') 
          {
            commandList.add
            (
              Commands.sequence
              (
                Commands.waitSeconds(0.05),
                s_Diffector.runOnce(() -> s_Diffector.goToAngle(0))
              )
            );
          }
          
          commandList.add(Commands.waitUntil(() -> !RobotContainer.coral));
          commandList.add(s_Coral.setStatusCommand(CoralManipulator.Status.DEFAULT));
          commandList.add(s_Diffector.moveToCommand(Presets.coralStowPosition));
          break;

        case 'c':
          nextPath = FieldUtils.loadPath(Constants.Auto.autoMap.get(splitCommand).pathName);
          ArmPos armPos = splitCommand.charAt(1) == 'r' ? Presets.coralIntakePosition.stbd() : Presets.coralIntakePosition.port();

          Pathfinding.setStartPosition(prevEndPoint);

          commandList.add
          (
            Commands.parallel
            (
              AutoBuilder.pathfindThenFollowPath(nextPath, stationConstraints),
              Commands.waitSeconds(0.15).andThen(s_Diffector.moveAndWaitCommand(armPos))
            )
          );

          prevEndPoint = nextPath.getWaypoints().get(nextPath.getWaypoints().size() - 1).anchor();    

          commandList.add(s_Coral.setStatusCommand(CoralManipulator.Status.INTAKE));
          commandList.add(Commands.waitUntil(() -> RobotContainer.coral));
          commandList.add(s_Diffector.moveToCommand(Presets.algaeStowPosition));
          break;
      
        default:
          autoMapValue = Constants.Auto.autoMap.get(splitCommand);

          if (autoMapValue.pathName != null) 
          {       
            nextPath = FieldUtils.loadPath(autoMapValue.pathName);
    
            Pathfinding.setStartPosition(prevEndPoint);
            
            commandList.add(AutoBuilder.pathfindThenFollowPath(nextPath, defaultConstraints));
            prevEndPoint = nextPath.getWaypoints().get(nextPath.getWaypoints().size() - 1).anchor();    
          }

          if (autoMapValue.command != null) 
          {
            commandList.add(autoMapValue.command.get());
          }
          break;
      }
    }

    return new SequentialCommandGroup(commandList.toArray(Command[]::new)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public static void displayPose(Pose2d pose, Rotation2d rotation)
  {
    SD.IO_POSE_X.put(pose.getX());
    SD.IO_POSE_Y.put(pose.getY());
    SD.IO_POSE_R.put(rotation.getDegrees());
  }

  public static void displayPose(Pose2d pose) 
  {
    displayPose(pose, pose.getRotation());
  }
}
