package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsytem extends SubsystemBase{
    public enum hightes{
            level, 
            stattion, 
            levcel2, 
            level3, 
            level4
    }
    private SparkMax elevatormotor = new SparkMax(1, MotorType.kBrushless);

}
