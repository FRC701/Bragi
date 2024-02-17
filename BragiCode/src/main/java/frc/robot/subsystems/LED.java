// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private CANdle m_CaNdle;

  // private final int LedCount = 60;

  public static LedState mLedState;

  public LED() {
    m_CaNdle = new CANdle(9);

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
    m_CaNdle.setLEDs(255, 255, 0);
  }

  public void Red() {
    m_CaNdle.setLEDs(255, 0, 0);
  }

  public void Purple() {
    m_CaNdle.setLEDs(153, 51, 255);
  }

  public void Blue() {
    m_CaNdle.setLEDs(0, 0, 255);
  }

  public void Green() {
    m_CaNdle.setLEDs(0, 255, 0);
  }

  public void Pink() {
    m_CaNdle.setLEDs(255, 51, 255);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("LEDState", mLedState.toString());

    RunLedState();

    // This method will be called once per scheduler run
  }
}
