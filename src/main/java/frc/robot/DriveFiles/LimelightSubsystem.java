package frc.robot.DriveFiles;

import java.util.Optional;

import org.opencv.core.Mat;

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
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase{
    
public SwerveSubsystem s_swerve;
// public NetworkTable networkTables;
// public IntegerSubscriber pipeline;
// public IntegerPublisher pipelinePublisher;
// public double targetHeight = 1.450975;
// public double cameraHeight = 1.2065;
public double xAng, yAng, ySpeed, xSpeed, turningSpeed, targetID, targetArea, correctionX, correctionY, correctionT, distanceX, distanceY, yAngToRadians, xAngToRadians;
public double[] localizedPose;
public double[] botPose_targetSpace;
public ProfiledPIDController thetaPIDController;
public ProfiledPIDController lateralPIDController;
public boolean autoDrive = false;
public boolean  hasTargets;
public String limelightName = "limelight";

public SlewRateLimiter tLimiter, xLimiter, yLimiter;

    public LimelightSubsystem(SwerveSubsystem s_swerve){
        this.s_swerve = s_swerve;
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);
        // networkTables = NetworkTableInstance.getDefault().getTable("limelight");
        // pipeline = networkTables.getIntegerTopic("limelight.getpipe").subscribe(0);
        // pipelinePublisher = networkTables.getIntegerTopic("limelight.getpipeline").publish();
        
        
        thetaPIDController = new ProfiledPIDController(Constants.limelightConstants.thetakP, Constants.limelightConstants.thetakI, Constants.limelightConstants.thetakD, Constants.AutoConstants.kThetaControllerConstraints);
        lateralPIDController = new ProfiledPIDController(Constants.limelightConstants.linearkP, Constants.limelightConstants.linearkI, Constants.limelightConstants.linearkD, Constants.AutoConstants.kLinearConstraints);
        tLimiter = new SlewRateLimiter(Constants.AutoConstants.kMaxAngularAccelerationUnitsPerSecond);
        xLimiter = new SlewRateLimiter(Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        yLimiter = new SlewRateLimiter(Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

        setThetaPID();
        setLinearPID();
    }

    public Optional<Pose2d> getPoseFromAprilTags() {
        double[] botpose = localizedPose;
        if(botpose.length < 7 || targetID == -1) return Optional.empty();
        return Optional.of(new Pose2d(botpose[0], botpose[1], new Rotation2d(botpose[5])));
    }
    public void setThetaPID(){
        thetaPIDController.setGoal(0);
        thetaPIDController.setTolerance(Math.toRadians(2));
        }
    public void setLinearPID(){
        lateralPIDController.setGoal(90-18.5);
        lateralPIDController.setTolerance(2);//degrees
    }



    public Command autoDriveToggle()
    {
        return Commands.runOnce(()->
        {
           autoDrive = !autoDrive;
        });
    }


    public void autoDrive()
    {
        
        if(hasTargets && (!lateralPIDController.atGoal() || !thetaPIDController.atGoal()))
            {
                ySpeed = -1* lateralPIDController.calculate(90-yAng);
                xSpeed = 0;
                turningSpeed = thetaPIDController.calculate(xAng);
            }else
            {
                ySpeed = 0;
                xSpeed = 0;
                turningSpeed = 0;
            }
    }
    
    
    @Override
    public void periodic(){
        // distanceX = ((targetHeight-cameraHeight) / (Math.tan(yAng)));//inches
        // distanceY = distanceX * Math.tan(xAng);//inches
        // distanceX = botPose_targetSpace[0];
        // distanceY = Math.abs(botPose_targetSpace[2]);

        botPose_targetSpace = LimelightHelpers.getBotPose_TargetSpace(limelightName);
        localizedPose = s_swerve.isRedAlliance ? LimelightHelpers.getBotPose_wpiRed(limelightName) : LimelightHelpers.getBotPose_wpiBlue(limelightName);
        xAng = LimelightHelpers.getTX(limelightName);
        xAngToRadians = Math.toRadians(xAng);
        yAng = LimelightHelpers.getTY(limelightName)+ Constants.limelightConstants.angleOffset;
        yAngToRadians = Math.toRadians(yAng);
        hasTargets = LimelightHelpers.getTV(limelightName);
        targetID = LimelightHelpers.getFiducialID(limelightName);
        targetArea = LimelightHelpers.getTA(limelightName);
        correctionY = (90-yAng) * lateralPIDController.getP() * Constants.AutoConstants.kMaxSpeedMetersPerSecond;//meters per second ;//meters
        correctionT = -1 * Math.toRadians(xAng * thetaPIDController.getP()) * Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond;//radians per second
        
        autoDrive();
        


// SmartDashboard.putNumber("Distance X", distanceX);
// SmartDashboard.putNumber("Distance Z", distanceY);
SmartDashboard.putNumber("Turning Speed", turningSpeed);
SmartDashboard.putNumber("TA Value", targetArea);
// SmartDashboard.putNumber("X Speed", xSpeed);
SmartDashboard.putNumber("Y Speed", ySpeed);
SmartDashboard.putNumber("TX Value", xAng);
SmartDashboard.putNumber("TY Value", yAng);


// SmartDashboard.putNumber("BotPose X", botPose_targetSpace[0]);
// SmartDashboard.putNumber("BotPose Y", botPose_targetSpace[1]);
// SmartDashboard.putNumber("BotPose Z", botPose_targetSpace[2]);



    }
}
