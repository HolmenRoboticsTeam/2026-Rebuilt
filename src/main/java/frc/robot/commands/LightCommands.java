// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;

/** Add your docs here. */
public class LightCommands {

  private static AddressableLED led = new AddressableLED(0);
  private static AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(14);

  static {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  /**
   * I don't know what this method does. It's 9:12, the night before comp.
   *
   * @param percentage the current percentage thought the given colors
   * @param colors the given colors
   * @return A command with the given logic
   */
  public static Command controlLights(Supplier<Double> percentage, Color... colors) {

    LEDPattern[] patterns = new LEDPattern[colors.length];
    for (int i = 0; i < colors.length; i++) {
      patterns[i] = LEDPattern.solid(colors[i]);
    }

    return Commands.run(
            () -> {
              double value = MathUtil.clamp(percentage.get(), 0.0, 1.0);
              int index = (int) Math.floor(value * (patterns.length - 1));
              patterns[index].applyTo(ledBuffer);
              led.setData(ledBuffer);
            })
        .ignoringDisable(true)
        .withName("LightControlCommand");
  }
}
