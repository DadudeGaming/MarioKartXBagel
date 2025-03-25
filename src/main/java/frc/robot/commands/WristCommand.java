// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Wrist;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class WristCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Wrist m_subsystem;
  private int m_PIDPositionIndex;

  Vector2[] wristAnglesArray;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WristCommand(Wrist subsystem, int PIDPositionIndex) {
    m_subsystem = subsystem;
    m_PIDPositionIndex = PIDPositionIndex;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristAnglesArray = new Vector2[8];
    wristAnglesArray[0] = WristConstants.PRESETS.STOWED;
    wristAnglesArray[1] = WristConstants.PRESETS.L1;
    wristAnglesArray[2] = WristConstants.PRESETS.L2;
    wristAnglesArray[3] = WristConstants.PRESETS.L3;
    wristAnglesArray[4] = WristConstants.PRESETS.L4;
    wristAnglesArray[5] = WristConstants.PRESETS.INTAKE;
    wristAnglesArray[6] = WristConstants.PRESETS.INTERMEDIATE;
    wristAnglesArray[7] = WristConstants.PRESETS.GROUND;
    m_subsystem.setPID(wristAnglesArray[m_PIDPositionIndex]);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // pivotsubsytem.setsetpoint(input)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.isAtGoal();
  }
}