package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LightSubsystem extends SubsystemBase{

public Spark lights;


public LightSubsystem(){
    lights = new Spark(9);

}


public void setLeds(int num)
{
    lights.set(num);
}


@Override
public void periodic(){
    // lights.set(0.27);
    // if(s_Swerve.allianceCheck()){
    //     lights.set(-0.25);
    // }else{
    //     lights.set(-0.23);
    // }

}



}
