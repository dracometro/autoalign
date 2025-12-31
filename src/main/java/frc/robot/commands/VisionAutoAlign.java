package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Limelight;
import org.littletonrobotics.junction.Logger;

public class VisionAutoAlign extends Command {
  private final Limelight vision;
  private final Drive drive;
  private final PIDController pidTurn = new PIDController(0.3, 0, 0);
  private final PIDController pidMove = new PIDController(0.08, 0, 0);
  private double[] wantedPose = new double[] {0, 1};

  public VisionAutoAlign(Limelight vision, Drive drive) {
    this.vision = vision;
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    pidMove.reset();
    pidTurn.reset();
  }

  @Override
  public void execute() {

    double offset = vision.getTagOffset();

    double rotation = pidTurn.calculate(offset, 0);
    if (Math.abs(offset) <= 0.2) {
      rotation = 0;
    }

    double targetAngle =
        Math.atan2(wantedPose[1] - vision.pose2dTag()[1], wantedPose[0] - vision.pose2dTag()[0]);

    double robotAngle = drive.getRotation().getRadians();

    double headingOffset = MathUtil.angleModulus(targetAngle - robotAngle);
    double distance =
        pidMove.calculate(
            Math.hypot(
                wantedPose[0] - vision.pose2dTag()[0], wantedPose[1] - vision.pose2dTag()[1]),
            0);

    // idk what this math is i hope work
    double velX = distance * Math.cos(headingOffset);
    double velY = distance * Math.sin(headingOffset);

    Logger.recordOutput("Vision/YOffset", velY);
    Logger.recordOutput("Vision/XOffset", velX);
    Logger.recordOutput("Vision/dist", distance);
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(velX, velY, rotation, drive.getRotation());
    if (vision.getHasTarget() && !(rotation > 0.1 && Math.abs(distance) > 1)) {
      drive.runVelocity(speeds);
    } else {
      drive.runVelocity(new ChassisSpeeds(0, 0, 0));
    }
  }
}
