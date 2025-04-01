// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.WristNotDiffy;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class WristCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final WristNotDiffy m_subsystem;
  private int m_PIDPositionIndex;

  double[] wristAnglesArray;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WristCommand(WristNotDiffy subsystem, int PIDPositionIndex) {
    m_subsystem = subsystem;
    m_PIDPositionIndex = PIDPositionIndex;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristAnglesArray = new double[8];
    wristAnglesArray[0] = WristConstants.PresetsNonDiffy.STOWED;
    wristAnglesArray[1] = WristConstants.PresetsNonDiffy.L1;
    wristAnglesArray[2] = WristConstants.PresetsNonDiffy.L2;
    wristAnglesArray[3] = WristConstants.PresetsNonDiffy.L3;
    wristAnglesArray[4] = WristConstants.PresetsNonDiffy.Climb;
    wristAnglesArray[5] = WristConstants.PresetsNonDiffy.INTAKE;
    wristAnglesArray[6] = WristConstants.PresetsNonDiffy.GROUND;
    wristAnglesArray[7] = WristConstants.PresetsNonDiffy.INTERMEDIATE;
    
    m_subsystem.setSetpoint(wristAnglesArray[m_PIDPositionIndex]);
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