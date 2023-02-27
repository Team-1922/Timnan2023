// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightEmitingDiode extends SubsystemBase {

  private final CANdle m_candle = new CANdle(1);

  private int idx = 0;
  /** Creates a new LED. */
  public LightEmitingDiode() {
  
  }

  @Override
  public void periodic() {
    m_candle.setLEDs(100, 100, 100);
    // idx += 1;
    // idx %= 68;
    // m_candle.setLEDs(0, 0, 0);
    // m_candle.setLEDs(255, 255, 255, 255, idx, 1);
    // This method will be called once per scheduler run
  }
}
