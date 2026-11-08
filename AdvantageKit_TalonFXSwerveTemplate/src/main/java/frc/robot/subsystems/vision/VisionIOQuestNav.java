package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.LinkedList;
import java.util.List;

public class VisionIOQuestNav implements VisionIO {
  private final QuestNav questNav = new QuestNav();

  public VisionIOQuestNav() {}

  // Tell quest where it is
  public void setPose(Pose3d pose) {
    questNav.setPose(pose);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Updates callbacks
    questNav.commandPeriodic();

    questNav.onConnected(() -> inputs.connected = true);
    questNav.onDisconnected(() -> inputs.connected = false);

    questNav.onTrackingAcquired(() -> SmartDashboard.putBoolean("QuestNav/Tracking", true));
    questNav.onTrackingLost(() -> SmartDashboard.putBoolean("QuestNav/Tracking", false));

    questNav
        .getBatteryPercent()
        .ifPresent(b -> SmartDashboard.putNumber("QuestNav/BatteryPercent", b));

    List<PoseObservation> poseObservations = new LinkedList<>();

    inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);

    PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

    for (PoseFrame questFrame : questFrames) {
      // isTracking just makes sure its not lying
      if (questFrame.isTracking() && inputs.connected) {
        Pose3d questPose = questFrame.questPose3d();
        double timestamp = questFrame.dataTimestamp();
        Pose3d robotPose = questPose.transformBy(robotToQuest.inverse());

        poseObservations.add(
            new PoseObservation(timestamp, robotPose, 0, 0, 0, PoseObservationType.QUESTNAV));
      }
    }
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }
  }
}
