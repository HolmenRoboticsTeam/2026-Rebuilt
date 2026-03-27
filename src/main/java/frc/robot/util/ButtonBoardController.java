// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;

/** Add your docs here. */
public class ButtonBoardController {

  private CommandGenericHID leftControls;
  private CommandGenericHID rightControls;

  private Map<Pair<Integer, Integer>, Trigger> buttonMap = new HashMap<>();

  public ButtonBoardController(int leftPort, int rightPort) {
    leftControls = new CommandGenericHID(leftPort);
    rightControls = new CommandGenericHID(rightPort);

    linkButtons();
  }

  public Trigger get(int row, int col) {
    return buttonMap.get(new Pair<Integer, Integer>(row, col));
  }

  public Trigger axisGreaterThan(int axis, double threshold) {
    if (axis < 2) {
      return leftControls.axisGreaterThan(axis, threshold);
    } else {
      return rightControls.axisGreaterThan(axis - 2, threshold);
    }
  }

  public Trigger axisLessThan(int axis, double threshold) {
    if (axis < 2) {
      return leftControls.axisLessThan(axis, threshold);
    } else {
      return rightControls.axisLessThan(axis - 2, threshold);
    }
  }

  public double getRawAxis(int axis) {
    if (axis < 2) {
      return leftControls.getRawAxis(axis);
    } else {
      return rightControls.getRawAxis(axis - 2);
    }
  }

  private void linkButtons() {
    // ########## Left Side ##########

    // Col 1
    buttonMap.put(
        new Pair<Integer, Integer>(1, 1), new Trigger(() -> leftControls.button(3).getAsBoolean()));
    // buttonMap.put(new Pair<Integer,Integer>(2, 1), new Trigger(() ->
    // leftControls.button(-1).getAsBoolean()));
    buttonMap.put(
        new Pair<Integer, Integer>(3, 1), new Trigger(() -> leftControls.button(2).getAsBoolean()));
    // buttonMap.put(new Pair<Integer,Integer>(4, 1), new Trigger(() ->
    // leftControls.button(-1).getAsBoolean()));

    // Col 2
    buttonMap.put(
        new Pair<Integer, Integer>(1, 2), new Trigger(() -> leftControls.button(4).getAsBoolean()));
    buttonMap.put(
        new Pair<Integer, Integer>(2, 2), new Trigger(() -> leftControls.button(5).getAsBoolean()));
    buttonMap.put(
        new Pair<Integer, Integer>(3, 2), new Trigger(() -> leftControls.button(1).getAsBoolean()));
    // buttonMap.put(new Pair<Integer,Integer>(4, 2), new Trigger(() ->
    // leftControls.button(-1).getAsBoolean()));

    // ########## Right Side ##########

    // Col 3
    buttonMap.put(
        new Pair<Integer, Integer>(1, 3),
        new Trigger(() -> rightControls.button(4).getAsBoolean()));
    buttonMap.put(
        new Pair<Integer, Integer>(2, 3),
        new Trigger(() -> rightControls.button(3).getAsBoolean()));
    buttonMap.put(
        new Pair<Integer, Integer>(3, 3),
        new Trigger(() -> rightControls.button(2).getAsBoolean()));
    // buttonMap.put(new Pair<Integer,Integer>(4, 3), new Trigger(() ->
    // rightControls.button(-1).getAsBoolean()));

    // Col 4
    buttonMap.put(
        new Pair<Integer, Integer>(1, 4),
        new Trigger(() -> rightControls.button(5).getAsBoolean()));
    // buttonMap.put(new Pair<Integer,Integer>(2, 4), new Trigger(() ->
    // rightControls.button(-1).getAsBoolean()));
    buttonMap.put(
        new Pair<Integer, Integer>(3, 4),
        new Trigger(() -> rightControls.button(1).getAsBoolean()));
    // buttonMap.put(new Pair<Integer,Integer>(4, 4), new Trigger(() ->
    // rightControls.button(-1).getAsBoolean()));
  }
}
