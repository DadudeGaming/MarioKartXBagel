// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class LowerCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_armSubsystem;
  private final TelescopeSubsystem m_TelescopeSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LowerCommand(ArmSubsystem armSubsystem, TelescopeSubsystem telescopeSubsystem) {
    m_armSubsystem = armSubsystem;

    m_TelescopeSubsystem = telescopeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem, telescopeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_armSubsystem.profiledArmController.getGoal().position + 10 < 90){
      m_armSubsystem.setArmAngle(m_armSubsystem.profiledArmController.getGoal().position + 10);
    } else {
      // m_armSubsystem.setArmAngle(90);
    }

    if(m_TelescopeSubsystem.telescopePID.getGoal().position - 10 > 0){
      m_TelescopeSubsystem.setTelescopeLength(m_TelescopeSubsystem.telescopePID.getGoal().position - 10);
    } else {
      m_TelescopeSubsystem.setTelescopeLength(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_TelescopeSubsystem.isAtGoal() && m_armSubsystem.isAtGoal();
  }
}
