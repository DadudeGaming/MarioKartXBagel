// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.TelescopeCommand;
import frc.robot.commands.WristCommand;

public class StateManager extends SubsystemBase {

  public static int robotState = 0;
  
    public StateManager() {
      Shuffleboard.getTab(getName()).addString("robotstate?", () -> String.valueOf(this.robotStateSupplier.getAsInt()));
    }
  
  
    public Command setRobotState(int state){
      return runOnce(() -> {
        robotState = state;
    });
  }


  public Command moveToPosition (int position, TelescopeSubsystem telescope, ArmSubsystem arm, WristNotDiffy wrist) {
    return defer(() -> {
      if (position != robotState || position == 0){
       return new SequentialCommandGroup(
                                        new TelescopeCommand(telescope, 0).alongWith(new WristCommand(wrist, 0)),
                                        new ArmCommand(arm, position),
                                        new TelescopeCommand(telescope, position).alongWith(new WristCommand(wrist, position)),
                                        setRobotState(position)
                                      );
    } else {
      return Commands.print("Already At Desired State");
    }
    });
   
  }

  public IntSupplier robotStateSupplier = () -> robotState;
  

  public Command goToState(int position, TelescopeSubsystem telescope, ArmSubsystem arm, WristNotDiffy wrist) {
    return defer(() -> {
      if (robotStateSupplier.getAsInt() == 6) {
      return  new SequentialCommandGroup(new PrintCommand("this works"),
                                 new TelescopeCommand(telescope, 7).alongWith(new ArmCommand(arm, 7).alongWith(new WristCommand(wrist, 7))),
                                 moveToPosition(position, telescope, arm, wrist)
                                 );
    } else {
      return moveToPosition(position, telescope, arm, wrist);
    }
    });
    // check if the robot is currently at intake and then move to the intermediate position 
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run, constantly, usually you'll put logging stuff here
    if(DriverStation.isDisabled()){
      robotState = 0;
    }
  }
}
