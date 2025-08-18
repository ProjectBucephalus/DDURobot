package frc.robot.util;

import java.util.ArrayList;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
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
    // Removes all whitespace characters from the single-String command phrases, ensures it's all lowercase, and then splits it into individual strings, which are stored in an array
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
            MathUtil.clamp(Double.parseDouble(splitCommand.substring(1, seperatorIndex)), 0.5, (FieldConstants.fieldCentre.getX()) - 0.5), 
            MathUtil.clamp(Double.parseDouble(splitCommand.substring(seperatorIndex + 1)), 0.5, FieldConstants.fieldWidth - 0.5)
          );

          Rotation2d rotationTarget = 
          splitCommand.contains(";") ? 
          new Rotation2d(Units.degreesToRadians(Double.parseDouble(splitCommand.substring(splitCommand.indexOf(";"))))) : 
          swerveStateSup.get().Pose.getRotation().plus(Rotation2d.k180deg);
          
          commandList.add(s_Swerve.poseLockDriveCommand(new AlliancePose2dSup(posTarget, rotationTarget), swerveStateSup));
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
          commandList.add(s_Coral.setSpeedCommand(Constants.Coral.forwardSpeed).until(s_Coral::getSensor));
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
}
