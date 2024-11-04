package frc.robot.Commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LightSubsystem;
import frc.robot.Subsystems.StateMachine;

public class NoneState extends Command {
    StateMachine s_StateMachine;
    LightSubsystem s_Lights;



    public NoneState(StateMachine s_StateMachine, LightSubsystem s_Lights)
    {
        this.s_StateMachine = s_StateMachine;
        this.s_Lights = s_Lights;
        addRequirements(s_StateMachine);
    }


    @Override
    public void initialize()
    {
        
    }
}
