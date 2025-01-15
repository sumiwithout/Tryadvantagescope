package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.Elevatorsubsystem;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorSubsytem extends SubsystemBase{
    public enum hightes{
            level, 
            stattion, 
            levcel2, 
            level3, 
            level4
    }
    private SparkMax elevatormotor = new SparkMax(20, MotorType.kBrushless);
    private SparkClosedLoopController elactorcontorller = elevatormotor.getClosedLoopController();
    private RelativeEncoder elevatEncoder = elevatormotor.getEncoder();

    private DCMotor eleccatorMotorModele = DCMotor.getNEO(20);
    private SparkMaxSim elevatorMotorSim;
      private SparkLimitSwitchSim elevatorLimitSwitchSim;

public ElevatorSubsytem(){
    elevatormotor.configure(Elevatorsubsystem.elevatorconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatEncoder.setPosition(0);

}
private void moveToSetpoint(){
//  will work on it more later
}

}
