// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.StateManager;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class OuttakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final EndEffectorSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public OuttakeCommand(EndEffectorSubsystem subsystem){
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(StateManager.robotState == 1){
      m_subsystem.setMotor(0.1);
    } else {
      m_subsystem.setMotor(0.3);
    }
      
    
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
    return false;
  }
}