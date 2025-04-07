package frc.robot.commands;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
 


public class ElevatorCommand extends Command{
  public ElevatorCommand(){
    addRequirements(RobotContainer.m_elevatorSubsystem);
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //L1 Setpoint
    if(RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_xButton)) {
      double L1PValue = Constants.ElevatorConstants.m_elevatorP;
      double L1IValue = Constants.ElevatorConstants.m_elevatorI;
      double L1DValue = Constants.ElevatorConstants.m_elevatorD;
      RobotContainer.m_elevatorSubsystem.setPIDValues(L1PValue, L1IValue, L1DValue, 
      Constants.ElevatorConstants.L1_PIDS.m_L1MinOutput, Constants.ElevatorConstants.L1_PIDS.m_L1MaxOutput);
      double L1Rotations = Constants.ElevatorConstants.L1_PIDS.m_L1Rotations;
      RobotContainer.m_elevatorSubsystem.setArmReference(L1Rotations, SparkBase.ControlType.kPosition);
    }
    //L2 Setpoint
    if(RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_bButton)) {
      double L2PValue = Constants.ElevatorConstants.m_elevatorP;
      double L2IValue = Constants.ElevatorConstants.m_elevatorI;
      double L2DValue = Constants.ElevatorConstants.m_elevatorD;
      RobotContainer.m_elevatorSubsystem.setPIDValues(L2PValue, L2IValue, L2DValue, 
      Constants.ElevatorConstants.L2_PIDS.m_L2MinOutput, Constants.ElevatorConstants.L2_PIDS.m_L2MaxOutput);
      double L2Rotations = Constants.ElevatorConstants.L2_PIDS.m_L2Rotations;
      RobotContainer.m_elevatorSubsystem.setArmReference(L2Rotations, SparkBase.ControlType.kPosition);
    }
    //L3 Setpoint
    if(RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_yButton)) {
      double L3PValue = Constants.ElevatorConstants.m_elevatorP;
      double L3IValue = Constants.ElevatorConstants.m_elevatorI;
      double L3DValue = Constants.ElevatorConstants.m_elevatorD;
      RobotContainer.m_elevatorSubsystem.setPIDValues(L3PValue, L3IValue, L3DValue, 
      Constants.ElevatorConstants.L3_PIDS.m_L3MinOutput, Constants.ElevatorConstants.L3_PIDS.m_L3MaxOutput);
      double L3Rotations = Constants.ElevatorConstants.L3_PIDS.m_L3Rotations;
      RobotContainer.m_elevatorSubsystem.setArmReference(L3Rotations, SparkBase.ControlType.kPosition); 
    }
    //Coral Station 
    if(RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_aButton)) {
      double StationPValue = Constants.ElevatorConstants.m_elevatorP;
      double StationIValue = Constants.ElevatorConstants.m_elevatorI;
      double StationDValue = Constants.ElevatorConstants.m_elevatorD;
      RobotContainer.m_elevatorSubsystem.setPIDValues(StationPValue, StationIValue, StationDValue, 
      Constants.ElevatorConstants.Station_PIDS.m_stationMinOutput, Constants.ElevatorConstants.Station_PIDS.m_stationMaxOutput);
      double StationRotations = Constants.ElevatorConstants.Station_PIDS.m_stationRotations;
      RobotContainer.m_elevatorSubsystem.setArmReference(StationRotations, SparkBase.ControlType.kPosition);
    }
    //Algae Setpoint
    if(RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_leftStickButton)) {
      double AlgaePValue = Constants.ElevatorConstants.m_elevatorP;
      double AlgaeIValue = Constants.ElevatorConstants.m_elevatorI;
      double AlgaeDValue = Constants.ElevatorConstants.m_elevatorD;
      RobotContainer.m_elevatorSubsystem.setPIDValues(AlgaePValue, AlgaeIValue, AlgaeDValue, 
      Constants.ElevatorConstants.Algae_PIDS.m_algaeMinOutput, Constants.ElevatorConstants.Algae_PIDS.m_algaeMaxOutput);
      double AlgaeRotations = Constants.ElevatorConstants.Algae_PIDS.m_algaeRotations;
      RobotContainer.m_elevatorSubsystem.setArmReference(AlgaeRotations, SparkBase.ControlType.kPosition);
    }
    //Zero Position Setpoint (and maybe Coral Station)
    if(RobotContainer.m_operatorController.getRawButtonPressed(Constants.OperatorConstants.JoystickButtons.m_rightStickButton)) {
      double ZeroPValue = Constants.ElevatorConstants.m_elevatorP;
      double ZeroIValue = Constants.ElevatorConstants.m_elevatorI;
      double ZeroDValue = Constants.ElevatorConstants.m_elevatorD;
      RobotContainer.m_elevatorSubsystem.setPIDValues(ZeroPValue, ZeroIValue, ZeroDValue, 
      Constants.ElevatorConstants.ZeroPIDS.m_zeroMinOutput, Constants.ElevatorConstants.ZeroPIDS.m_zeroMaxOutput);
      double ZeroRotations = Constants.ElevatorConstants.ZeroPIDS.m_zeroRotations;
      RobotContainer.m_elevatorSubsystem.setArmReference(ZeroRotations, SparkBase.ControlType.kPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override

  public boolean isFinished() {
    double current = RobotContainer.m_elevatorSubsystem.getCurrentValue();
    boolean stopMotors = false;

    if(current >= Constants.ElevatorConstants.MAX_CURRENT){
      stopMotors = true;
    }
  
    return stopMotors;
  }
}

