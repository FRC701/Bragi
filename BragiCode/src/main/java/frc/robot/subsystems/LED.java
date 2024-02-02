// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private AddressableLED m_led;

  private AddressableLEDBuffer m_ledBuffer;

  public static LedState mLedState;

  public LED() {
    m_led = new AddressableLED(9);
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();

    mLedState = LedState.S_Blue;
  }

  public enum LedState {
    S_Red,
    S_Purple,
    S_Blue,
    S_Green,
    S_Yellow,
    S_Pink
  }

  public void RunLedState() {
    switch (mLedState) {
      case S_Red:
        Red();
        break;
      case S_Purple:
        Purple();
        break;
      case S_Blue:
        Blue();
        break;
      case S_Green:
        Green();
        break;
      case S_Pink:
        Pink();
        break;
    }
  }

  public void Red() {
    for (var i = 0; i < m_ledBuffer.getLength(); ) {
      m_ledBuffer.setRGB(i, 255, 0, 0);
    }
    m_led.setData(m_ledBuffer);
  }

  public void Purple() {
    for (var i = 0; i < m_ledBuffer.getLength(); ) {
      m_ledBuffer.setRGB(i, 153, 51, 255);
    }
    m_led.setData(m_ledBuffer);
  }

  public void Blue() {
    for (var i = 0; i < m_ledBuffer.getLength(); ) {
      m_ledBuffer.setRGB(i, 0, 0, 255);
    }
    m_led.setData(m_ledBuffer);
  }

  public void Green() {
    for (var i = 0; i < m_ledBuffer.getLength(); ) {
      m_ledBuffer.setRGB(i, 0, 255, 0);
    }
    m_led.setData(m_ledBuffer);
  }

  public void Pink() {
    for (var i = 0; i < m_ledBuffer.getLength(); ) {
      m_ledBuffer.setRGB(i, 255, 51, 255);
    }
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
