package frc.robot.Commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.constants_Drive;
import frc.robot.Constants.constants_OI;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;


public class DriveCommand extends Command{
    

    
    private final SwerveSubsystem s_Swerve;
    private final LimelightSubsystem s_limelight;
    public final CommandXboxController opController;
    // public final CommandJoystick leftStick;
    // public final CommandJoystick rightStick;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean fieldOriented=false;
     public double ySpeed, xSpeed, turningSpeed;
     public double ll_zSpeed, ll_xSpeed, ll_turningSpeed;
    public ChassisSpeeds chassisSpeeds;

    



    // public DriveCommand(s_Swerve s_Swerve, CommandXboxController opController, CommandJoystick leftStick, CommandJoystick rightStick) {
        public DriveCommand(SwerveSubsystem s_Swerve, CommandXboxController opController, LimelightSubsystem s_limelight) {

                this.s_Swerve = s_Swerve;
                this.s_limelight = s_limelight;
                this.xLimiter = new SlewRateLimiter(constants_Drive.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.yLimiter = new SlewRateLimiter(constants_Drive.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.turningLimiter = new SlewRateLimiter(constants_Drive.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
                addRequirements(s_Swerve);
                this.opController = opController;
                // this.leftStick = leftStick;
                // this.rightStick = rightStick;
    }


    @Override
    public void initialize() {
     s_Swerve.faceAllFoward();
    }

 


    @Override
    public void execute() {
      
        // xSpeed = IsJoyStick? -leftStick.getX(): -opController.getLeftX();
        // ySpeed = IsJoyStick? -leftStick.getY(): -opController.getLeftY();
        // turningSpeed = IsJoyStick? -rightStick.getX(): -opController.getRightX();
        xSpeed = -opController.getLeftX();
        ySpeed = -opController.getLeftY();
        turningSpeed = -opController.getRightX();
        fieldOriented = s_Swerve.fieldOriented;


        
        SmartDashboard.putBoolean("fieldOriented", fieldOriented);


        xSpeed = Math.abs(xSpeed) > constants_OI.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > constants_OI.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > constants_OI.kDeadband ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * constants_Drive.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * constants_Drive.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * constants_Drive.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds;
        // if(s_limelight.autoDrive)
        // {
        //     if(s_limelight.hasTargets)
        //     {
        //         // chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, s_limelight.turningSpeed);
        //         // chassisSpeeds = new ChassisSpeeds(s_limelight.ySpeed, s_limelight.xSpeed, s_limelight.turningSpeed);
        //         chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, s_limelight.turningSpeed, s_Swerve.geRotation2d());
        //     }else
        //     {
        //         chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, turningSpeed, s_Swerve.geRotation2d());
        //     }


        // }
        // else if(fieldOriented){
        //     chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, turningSpeed, s_Swerve.geRotation2d());
        // }else {
        //     chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);
        // }
        
        
        if(s_limelight.autoDriveToggle && s_limelight.hasTargets)
        {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(s_limelight.ySpeed, xSpeed, s_limelight.turningSpeed, s_Swerve.geRotation2d());
        }else
        {
            if(fieldOriented)
            {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, turningSpeed, s_Swerve.geRotation2d());
            }else
            {
                chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);
            }
        }
        s_Swerve.setModuleStates(chassisSpeeds);
    }


    @Override
    public void end(boolean interrupted) {
        s_Swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }



}