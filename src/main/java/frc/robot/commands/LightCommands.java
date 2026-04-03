// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.HubShiftUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class LightCommands {

  private static final int ledLength = 18;
  private static AddressableLED led = new AddressableLED(0);
  private static AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(ledLength);
  private static Timer timer = new Timer();

  static {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  /**
   * I don't know what this method does. It's 9:12, the night before comp. (We stayed til 12:58 am)
   *
   * @param percentage the current percentage thought the given colors
   * @param colors the given colors
   * @return A command with the given logic
   */
  public static Command controlLights(
      Supplier<Double> percentage, boolean flashLastColor, Color... colors) {

    LEDPattern[] patterns = new LEDPattern[colors.length];
    for (int i = 0; i < colors.length; i++) {
      patterns[i] = LEDPattern.solid(colors[i]);
    }

    timer.start();

    return Commands.run(
            () -> {
              double value = MathUtil.clamp(percentage.get(), 0.0, 1.0);
              int index = (int) Math.floor(value * (patterns.length - 1));

              if (DriverStation.isDisabled()) {
                LEDPattern.solid(Color.kBlack).applyTo(ledBuffer);
                led.setData(ledBuffer);
                return;
              }

              // Are we flashing colors and is time to flip colors
              if (flashLastColor && index == 0 && timer.hasElapsed(0.15)) {

                // Reset the timer
                timer.reset();
                timer.start();

                // Color is not black, so set black and return
                if (!ledBuffer.getLED(0).equals(Color.kBlack)) {
                  Logger.recordOutput("Lights/Color", Color.kBlack);
                  LEDPattern.solid(Color.kBlack).applyTo(ledBuffer);
                  led.setData(ledBuffer);
                  return;
                }
              } else if (flashLastColor && index == 0) {
                return;
              }

              // Apply index color
              Logger.recordOutput("Lights/Color", colors[index]);
              patterns[index].applyTo(ledBuffer);
              led.setData(ledBuffer);
            })
        .ignoringDisable(true)
        .withName("LightControlCommand");
  }

  public static Command standard() {
    return Commands.either(

            // Is active shift, about to be inactive
            LightCommands.controlLights(
                    () ->
                        MathUtil.clamp(
                                HubShiftUtil.getOfficialShiftInfo().remainingTime(), 0.0, 25.0)
                            / 25.0,
                    true,
                    Color.kRed,
                    Color.kPaleVioletRed,
                    Color.kBlack,
                    Color.kBlack,
                    Color.kBlack)
                .until(() -> HubShiftUtil.getOfficialShiftInfo().remainingTime() < 0.1),

            // Is inactive shift, about to be active
            LightCommands.controlLights(
                    () ->
                        MathUtil.clamp(
                                HubShiftUtil.getOfficialShiftInfo().remainingTime(), 0.0, 25.0)
                            / 25.0,
                    true,
                    Color.kGreen,
                    Color.kPaleGreen,
                    Color.kBlack,
                    Color.kBlack,
                    Color.kBlack)
                .until(() -> HubShiftUtil.getOfficialShiftInfo().remainingTime() < 0.1),
            () -> HubShiftUtil.getOfficialShiftInfo().active())
        .repeatedly()
        .withName("Lights_Controller");
  }
}
