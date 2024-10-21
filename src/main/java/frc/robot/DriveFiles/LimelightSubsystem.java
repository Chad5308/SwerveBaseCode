package frc.robot.DriveFiles;

import java.util.Optional;

import org.opencv.core.Mat;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
import frc.robot.LimelightHelpers.LimelightResults;

public class LimelightSubsystem extends SubsystemBase{
    
public SwerveSubsystem s_swerve;
public NetworkTable networkTables;
public IntegerSubscriber pipeline;
public IntegerPublisher pipelinePublisher;
public double xAng, yAng, zSpeed, xSpeed, turningSpeed, targetID, targetArea, correctionX, correctionZ, correctionT, distanceX, distanceY, yAngToRadians, xAngToRadians;
public double[] localizedPose;
public double[] botPose_targetSpace, targetPose_robotSpace;
public ProfiledPIDController thetaPIDController;
public ProfiledPIDController xPIDController, zPIDController;
public boolean autoDrive = false;
public boolean  hasTargets;
public String limelightName = "limelight";

public LimelightResults r_limelight;

// public Apri fieldLayout;

public SlewRateLimiter tLimiter, xLimiter, zLimiter;

    public LimelightSubsystem(SwerveSubsystem s_swerve){
        this.s_swerve = s_swerve;
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);
        networkTables = NetworkTableInstance.getDefault().getTable("limelight");
        pipelinePublisher = networkTables.getIntegerTopic("limelight.getpipeline").publish();
        
        
        thetaPIDController = new ProfiledPIDController(Constants.limelightConstants.thetakP, Constants.limelightConstants.thetakI, Constants.limelightConstants.thetakD, Constants.AutoConstants.kThetaControllerConstraints);
        xPIDController = new ProfiledPIDController(Constants.limelightConstants.linearkP, Constants.limelightConstants.linearkI, Constants.limelightConstants.linearkD, Constants.AutoConstants.kLinearConstraints);
        zPIDController = new ProfiledPIDController(Constants.limelightConstants.linearkP, Constants.limelightConstants.linearkI, Constants.limelightConstants.linearkD, Constants.AutoConstants.kLinearConstraints);
        tLimiter = new SlewRateLimiter(Constants.AutoConstants.kMaxAngularAccelerationUnitsPerSecond);
        xLimiter = new SlewRateLimiter(Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        zLimiter = new SlewRateLimiter(Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

      setPIDControllers();

        LimelightHelpers.LimelightResults r_limelight = new LimelightResults();

        // fieldLayout = new AprilTagFieldLayout(AprilTagFields.k2024Crescendo.);
    }

    public Optional<Pose2d> getPoseFromAprilTags() {
        double[] botpose = localizedPose;
        if(botpose.length < 7 || targetID == -1) return Optional.empty();
        return Optional.of(new Pose2d(botpose[0], botpose[1], new Rotation2d(botpose[5])));
    }
  

    public void setPIDControllers()
    {
        thetaPIDController.setGoal(0);//radians
        thetaPIDController.setTolerance(Math.toRadians(2)); //radians

        xPIDController.setGoal(0);//meters
        xPIDController.setTolerance(0.1);//meters

        zPIDController.setGoal(1.5);//meters
        zPIDController.setTolerance(0.25);//meters
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
        LimelightHelpers.setPipelineIndex(limelightName, 0);

        // if(hasTargets && (!xPIDController.atGoal() || !thetaPIDController.atGoal()))
        //     {
        //         zSpeed = -1* xPIDController.calculate(90-yAng);
        //         xSpeed = 0;
        //         turningSpeed = thetaPIDController.calculate(xAng);
        //     }else
        //     {
        //         zSpeed = 0;
        //         xSpeed = 0;
        //         turningSpeed = 0;
        //     }

            if(hasTargets && (!xPIDController.atGoal() || !zPIDController.atGoal() || !thetaPIDController.atGoal()))
            {
                // zSpeed = -1 * zPIDController.calculate(correctionZ);
                // xSpeed = -1 * xPIDController.calculate(correctionX);
                turningSpeed = thetaPIDController.calculate(xAngToRadians);
            }else
            {
                zSpeed = 0;
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
<<<<<<< HEAD

=======
        // correctionX = -1* targetArea * xPIDController.getP() * Constants.AutoConstants.kMaxSpeedMetersPerSecond;//meters per second
        
>>>>>>> aef94c987a29628b3542179bc09835b2cef9d5cd
        botPose_targetSpace = LimelightHelpers.getBotPose_TargetSpace(limelightName);
        targetPose_robotSpace = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
        localizedPose = s_swerve.isRedAlliance ? LimelightHelpers.getBotPose_wpiRed(limelightName) : LimelightHelpers.getBotPose_wpiBlue(limelightName);
        xAng = LimelightHelpers.getTX(limelightName);
        xAngToRadians = Math.toRadians(xAng);
        yAng = LimelightHelpers.getTY(limelightName);
        yAngToRadians = Math.toRadians(yAng);
        hasTargets = LimelightHelpers.getTV(limelightName);
        targetID = LimelightHelpers.getFiducialID(limelightName);
        targetArea = LimelightHelpers.getTA(limelightName);
        // correctionY = (90-yAng) * xPIDController.getP() * Constants.AutoConstants.kMaxSpeedMetersPerSecond;//meters per second ;//meters
        correctionT = -1 * Math.toRadians(xAng * thetaPIDController.getP()) * Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond;//radians per second
        autoDrive();
        
        correctionX = targetPose_robotSpace[0];
        correctionZ = targetPose_robotSpace[2];




// SmartDashboard.putNumber("Distance X", distanceX);
// SmartDashboard.putNumber("Distance Z", distanceY);
SmartDashboard.putNumber("Turning Speed", turningSpeed);
SmartDashboard.putNumber("TA Value", targetArea);
// SmartDashboard.putNumber("X Speed", xSpeed);
SmartDashboard.putNumber("Y Speed", zSpeed);
SmartDashboard.putNumber("TX Value", xAng);
SmartDashboard.putNumber("TY Value", yAng);


// SmartDashboard.putNumber("BotPose X", botPose_targetSpace[0]);
// SmartDashboard.putNumber("BotPose Y", botPose_targetSpace[1]);
// SmartDashboard.putNumber("BotPose Z", botPose_targetSpace[2]);



    }
}
