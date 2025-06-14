package frc.robot.util;

import java.util.ArrayList;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralRoller;

public class AutoFactories 
{
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

          Translation2d finalPosTarget = posTarget;
          Rotation2d rotationTarget = splitCommand.contains(";") ? new Rotation2d(Units.degreesToRadians(Double.parseDouble(splitCommand.substring(splitCommand.indexOf(";"))))) : swerveStateSup.get().Pose.getRotation();
          commandList.add(s_Swerve.poseLockDriveCommand(new AlliancePose2dSup(finalPosTarget, rotationTarget), swerveStateSup));
          break;

        case 'w':
          commandList.add(Commands.waitSeconds(Double.parseDouble(splitCommand.substring(1))));
          break;

        case 't':
          double targetMatchTimeElapsed = Double.parseDouble(splitCommand.substring(1));

          commandList.add(Commands.waitUntil(() -> Timer.getMatchTime() < (15 - targetMatchTimeElapsed)));
          break;

        case 'r':
          commandList.add(s_Swerve.poseLockDriveCommand(new AlliancePose2dSup(FieldConstants.getLineup(splitCommand)), swerveStateSup));
          commandList.add(Commands.waitSeconds(0.1));
          commandList.add(s_Coral.smartRunCommand(Constants.Coral.forwardSpeed));
          break;

        case 'c':
          commandList.add(s_Swerve.poseLockDriveCommand(new AlliancePose2dSup(FieldConstants.getLineup(splitCommand)), swerveStateSup));
          commandList.add(Commands.waitUntil(s_Coral::getSensor));
          break;
      
        default:
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
