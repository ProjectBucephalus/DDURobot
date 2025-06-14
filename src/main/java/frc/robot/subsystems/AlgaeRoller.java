package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;
import frc.robot.util.SD;

public class AlgaeRoller extends SubsystemBase
{
  private TalonFX m_Algae;
  private DigitalInput io_Sensor;

  public AlgaeRoller()
  {
    m_Algae = new TalonFX(IDConstants.AlgaeRollerID);
    io_Sensor = new DigitalInput(IDConstants.AlgaeSensorDIO);
  }

  public void setSpeed(double speed)
  {
    m_Algae.set(speed);
  }

  public boolean getSensor()
  {
    return io_Sensor.get();
  }
   
  public Command runCommand(double speed)
  {
    return startEnd(() -> setSpeed(speed), () -> setSpeed(0));
  }

  public Command smartRunCommand(double speed)
  {
    return runCommand(speed).until(this::getSensor);
  }

  @Override
  public void periodic() 
  {
    SD.ALGAE_ROLLER.put(m_Algae.get());
  }
}
