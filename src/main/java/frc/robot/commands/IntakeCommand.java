// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.EndEffectorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final EndEffectorSubsystem m_subsystem;

  private int wasJustStalled = 0;

  double[] pivotAnglesArray;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCommand(EndEffectorSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wasJustStalled = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_subsystem.setMotor(0.3);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotor(0);
    m_subsystem.intakeMode = !m_subsystem.intakeMode;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_subsystem.intakeMotor.getTorqueCurrent().getValueAsDouble() > IntakeConstants.stopCurrent){
      if(wasJustStalled < 5){
        wasJustStalled++;
        return false;
      }
      return true;
    }
    return false;
  }
}