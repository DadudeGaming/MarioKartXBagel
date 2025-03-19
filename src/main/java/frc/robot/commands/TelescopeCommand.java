// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.TelescopeConstants;
import frc.robot.subsystems.TelescopeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TelescopeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TelescopeSubsystem m_subsystem;
  private int m_PIDPositionIndex;

  double[] pivotAnglesArray;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TelescopeCommand(TelescopeSubsystem subsystem, int PIDPositionIndex) {
    m_subsystem = subsystem;
    m_PIDPositionIndex = PIDPositionIndex;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivotAnglesArray = new double[5];
    pivotAnglesArray[0] = TelescopeConstants.TelescopeLengths.Stowed;
    pivotAnglesArray[1] = TelescopeConstants.TelescopeLengths.L1;
    pivotAnglesArray[2] = TelescopeConstants.TelescopeLengths.L2;
    pivotAnglesArray[3] = TelescopeConstants.TelescopeLengths.L3;
    pivotAnglesArray[4] = TelescopeConstants.TelescopeLengths.L4;
    m_subsystem.setTelescopeLength(pivotAnglesArray[m_PIDPositionIndex]);
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