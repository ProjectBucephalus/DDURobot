package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

public class CoralRoller extends SubsystemBase
{
  private TalonFX m_Coral;
  private DigitalInput io_Sensor;

  public CoralRoller()
  {
    m_Coral = new TalonFX(IDConstants.coralRollerID);
    io_Sensor = new DigitalInput(IDConstants.coralSensorDIO);
  }

  public boolean getSensor()
    {return io_Sensor.get();}
   
  public Command setSpeedCommand(double speed)
    {return this.run(() -> m_Coral.set(speed));}

  @Override
  public void periodic() {}
}
