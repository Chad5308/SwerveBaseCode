package frc.robot.DriveFiles;

import java.util.Optional;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase{
    
public SwerveSubsystem s_swerve;
public NetworkTable networkTables;
public IntegerSubscriber pipeline;
public IntegerPublisher pipelinePublisher;
// public double targetHeight = 1.450975;
// public double cameraHeight = 1.2065;
public double xAng, yAng, hasTargets, ySpeed, xSpeed, turningSpeed, targetID;
public double correctionX, correctionY, correctionT;
public double distanceX, distanceY;
public double yAngToRadians, xAngToRadians;
public double[] localizedPose;
public double[] botPose_targetSpace;
public ProfiledPIDController thetaPIDController;
public ProfiledPIDController lateralPIDController;
public boolean autoAlign = false;

public SlewRateLimiter tLimiter, xLimiter, yLimiter;

    public LimelightSubsystem(SwerveSubsystem s_swerve){
        this.s_swerve = s_swerve;

// NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);



        networkTables = NetworkTableInstance.getDefault().getTable("limelight");

        pipeline = networkTables.getIntegerTopic("limelight.getpipe").subscribe(0);
        pipelinePublisher = networkTables.getIntegerTopic("limelight.getpipeline").publish();
        thetaPIDController = new ProfiledPIDController(Constants.limelightConstants.thetakP, Constants.limelightConstants.thetakI, Constants.limelightConstants.thetakD, Constants.AutoConstants.kThetaControllerConstraints);
        lateralPIDController = new ProfiledPIDController(Constants.limelightConstants.linearkP, Constants.limelightConstants.linearkI, Constants.limelightConstants.linearkD, Constants.AutoConstants.kLinearConstraints);
        tLimiter = new SlewRateLimiter(Constants.AutoConstants.kMaxAngularAccelerationUnitsPerSecond);
        xLimiter = new SlewRateLimiter(Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        yLimiter = new SlewRateLimiter(Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    }

    public Optional<Pose2d> getPoseFromAprilTags() {
        double[] botpose = localizedPose;
        if(botpose.length < 7 || targetID == -1) return Optional.empty();
        return Optional.of(new Pose2d(botpose[0], botpose[1], new Rotation2d(botpose[5])));
    }
    public void setThetaPID(){
        thetaPIDController.setGoal(0);
        thetaPIDController.setTolerance(Math.toRadians(5));
        }
    public void setLinearPID(){
        lateralPIDController.setTolerance(0.25);//meters
    }


    public Command alignRobot()
    {
        return Commands.runOnce(()-> 
        {
            autoAlign = true;
        });
    }
    public Command stopAlign()
    {
        return Commands.runOnce(()-> 
        {
            autoAlign = false;
        });
    }

    public void autoAlign()
    {
        if(!thetaPIDController.atGoal())
            {
                turningSpeed = -correctionT;
                xSpeed = 0;
                ySpeed = 0;
            }else
            {
                turningSpeed = 0;
                xSpeed = 0;
                ySpeed = 0;
            }
    }

    public void autoDrive()
    {
        if(!thetaPIDController.atGoal() && !lateralPIDController.atGoal())
            {
                turningSpeed = -correctionT;
                xSpeed = 0;
                ySpeed = 0;
            }else
            {
                turningSpeed = 0;
                xSpeed = 0;
                ySpeed = 0;
            }
    }
    
    
    @Override
    public void periodic(){
        // if (s_swerve.allianceCheck() == true) {
    //             localizedPose = networkTables.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    // } else {
        //         localizedPose = networkTables.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        //     }
        
        botPose_targetSpace = networkTables.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        localizedPose = networkTables.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        xAng = networkTables.getEntry("tx").getDouble(0);
        yAng = networkTables.getEntry("ty").getDouble(0) + Constants.limelightConstants.angleOffset;
        yAngToRadians = Math.toRadians(yAng);
        xAngToRadians = Math.toRadians(xAng);
        hasTargets = networkTables.getEntry("tv").getDouble(0);
        targetID = networkTables.getEntry("tid").getDouble(0);
        // distanceX = ((targetHeight-cameraHeight) / (Math.tan(yAng)));//inches
        // distanceY = distanceX * Math.tan(xAng);//inches
        // distanceX = botPose_targetSpace[0];
        // distanceY = Math.abs(botPose_targetSpace[2]);
        correctionX = lateralPIDController.calculate(distanceX);//meters
        correctionY = lateralPIDController.calculate(distanceY);//meters
        correctionT = Math.toRadians(xAng * thetaPIDController.getP()) * Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond;//radians
        
        autoAlign();
       
        


// SmartDashboard.putNumber("Distance X", distanceX);
// SmartDashboard.putNumber("Distance Z", distanceY);
SmartDashboard.putNumber("Turning Speed", turningSpeed);
// SmartDashboard.putNumber("X Speed", ySpeed);
// SmartDashboard.putNumber("Y Speed", xSpeed);
SmartDashboard.putNumber("TX Value", xAng);
// SmartDashboard.putNumber("TY Value", yAng);
// SmartDashboard.putNumber("TY degrees", Math.toDegrees(yAng));


// SmartDashboard.putNumber("BotPose X", botPose_targetSpace[0]);
// SmartDashboard.putNumber("BotPose Y", botPose_targetSpace[1]);
// SmartDashboard.putNumber("BotPose Z", botPose_targetSpace[2]);



    }
}
