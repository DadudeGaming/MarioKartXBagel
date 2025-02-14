// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_subsystem;

  private double speed;
  /**
   * Creates a new ExampleCommand.
   * 
   * 
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmUpCommand(IntakeSubsystem subsystem, double input) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    speed = input;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    if(!m_subsystem.hanging)
      m_subsystem.setMotor(speed);
    else
      m_subsystem.setMotor(-speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.hanging = !m_subsystem.hanging;
    m_subsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(/*motor at 90 degrees*/)
      return true;
    return false;
  }
}
