// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private CANdle m_CaNdle;

  //  private final int LedCount = 60;
  private CANdleConfiguration m_config = new CANdleConfiguration();

  public static LedState mLedState;

  public LED() {
    m_CaNdle = new CANdle(9);
    m_config.stripType = LEDStripType.RGB;
    m_CaNdle.configAllSettings(m_config);
    mLedState = LedState.Default;
  }

  public enum LedState {
    S_Red,
    S_Purple,
    S_Blue,
    S_Green,
    S_Pink,
    Default
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
      case Default:
        Yellow();
        break;
    }
  }

  public void Yellow() {
    m_CaNdle.setLEDs(255, 255, 0, 0, 0, Constants.kCandleLedCount);
  }

  public void Red() {
    m_CaNdle.setLEDs(255, 0, 0, 0, 0, Constants.kCandleLedCount);
  }

  public void Purple() {
    m_CaNdle.setLEDs(153, 51, 255, 0, 0, Constants.kCandleLedCount);
  }

  public void Blue() {
    m_CaNdle.setLEDs(0, 0, 255, 0, 0, Constants.kCandleLedCount);
  }

  public void Green() {
    m_CaNdle.setLEDs(0, 255, 0, 0, 0, Constants.kCandleLedCount);
  }

  public void Pink() {
    m_CaNdle.setLEDs(255, 51, 255, 0, 0, Constants.kCandleLedCount);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("LEDState", mLedState.toString());

    RunLedState();

    // This method will be called once per scheduler run
  }
}
