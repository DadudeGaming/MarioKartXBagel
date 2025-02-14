// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class PivotCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PivotSubsystem m_subsystem;
  private int m_PIDPositionIndex;

  double[] pivotAnglesArray;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PivotCommand(PivotSubsystem subsystem, int PIDPositionIndex) {
    m_subsystem = subsystem;
    m_PIDPositionIndex = PIDPositionIndex;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivotAnglesArray = new double[5];
    pivotAnglesArray[0] = PivotConstants.PivotAngles.firstPivotAngle;
    pivotAnglesArray[1] = PivotConstants.PivotAngles.secondPivotAngle;
    pivotAnglesArray[2] = PivotConstants.PivotAngles.thirdPivotAngle;
    pivotAnglesArray[3] = PivotConstants.PivotAngles.fourthPivotAngle;
    pivotAnglesArray[4] = PivotConstants.PivotAngles.fifthPivotAngle;
    m_subsystem.setPivotAngle(pivotAnglesArray[m_PIDPositionIndex]);
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
    return Math.abs(m_subsystem.encoderInDegrees() - pivotAnglesArray[m_PIDPositionIndex]) < PivotConstants.precisionInDegrees;
  }
}
