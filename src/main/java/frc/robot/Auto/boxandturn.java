package frc.robot.Auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;

public class boxandturn extends SequentialCommandGroup {
    public boxandturn(RobotContainer robot) {
        PathPlannerTrajectory trajectory1 =
        PathPlanner.generatePath(
            new PathConstraints(1.5, AutoConstants.kMaxAccelerationMetersPerSecondSquared),
            new PathPoint(
                new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
                Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(0)),
            new PathPoint(
                new Translation2d(Units.inchesToMeters(178), Units.inchesToMeters(0)),
                Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(0)));
    
}
}
