package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class Pulse extends SubsystemBase
{

  public Pulse() {
    initStrip(0, 60);
    //initLights(m_lightBar, m_lightBarBuffer, 2, 8);
  }

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private AddressableLED m_ledStrip;
  private AddressableLED m_lightBar;
  private AddressableLEDBuffer m_ledStripBuffer;
  private AddressableLEDBuffer m_lightBarBuffer;

  @Override
  public void periodic() {
    m_ledStrip.setData(m_ledStripBuffer);
    // m_lightBar.setData(m_lightBarBuffer);

  }

  private void initStrip(int port, int length) {
    m_ledStrip = new AddressableLED(port);
    m_ledStripBuffer = new AddressableLEDBuffer(length);
    m_ledStrip.setLength(length);
    m_ledStrip.start();
  }
  private void initBar(int port, int length) {
    m_lightBar = new AddressableLED(port);
    m_lightBarBuffer = new AddressableLEDBuffer(length);
    m_lightBar.setLength(length);
    m_lightBar.start();
  }

  private void setColors(AddressableLEDBuffer lightBuffer, Color... c) {
    for(int i = 0; i < lightBuffer.getLength(); i++) {
      lightBuffer.setLED(i, c[i % c.length]);
    }
  }

  private void lightsOff(AddressableLEDBuffer lightBuffer) {
    setColors(lightBuffer, new Color(0, 0, 0));
  }

  public Command runRedBlue(){
    return startEnd(
      () -> setColors(m_ledStripBuffer, new Color(255, 0, 0), new Color(0, 0, 255)),
      () -> lightsOff(m_ledStripBuffer));
  }

  public Command runLightsOff() {
    return run(() -> lightsOff(m_ledStripBuffer));
  }

}

