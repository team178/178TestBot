package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {

  
  private AddressableLEDBuffer m_lightBarBuffer;
  private AddressableLED m_lightBar;

  public Lights() {
    initBar();
  }
  

  @Override
  public void periodic() {
    m_lightBar.setData(m_lightBarBuffer);
  }

  private void initBar() {
    m_lightBar = new AddressableLED(0);
    m_lightBarBuffer = new AddressableLEDBuffer(8);
    m_lightBar.setLength(m_lightBarBuffer.getLength());
    m_lightBar.start();
  }

  private void setColor(Color c) {
    for(int i = 0; i < m_lightBarBuffer.getLength(); i++) {
      // Sets the specified LED to the Color inputted
      m_lightBarBuffer.setLED(i, c);
    }
  }
  
  private void off() {
    setColor(new Color(0, 0, 0));
  }

  
  private void yellow() {
    setColor(new Color(255, 255, 0));
  }

  private void purple() {
    setColor(new Color(150, 0, 255));
  }

  public Command runOff() {
    return run(() -> off());
  }

  public Command runYellow() {
    return run(() -> yellow());
  }

  public Command runPurple() {
    return run(() -> purple());
  }
}