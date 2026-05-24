package frc.robot.game;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.telemetry.Telemetry;

public class TransitionTracker {
  // Times derived from game manual page 36
  private static final double TELEOP_TOTAL = 2 * 60 + 20; // 140s
  private static final double TRANSITION_END = 10;
  private static final double PHASE_1_END = 35;
  private static final double PHASE_2_END = 60;
  private static final double PHASE_3_END = 85;
  private static final double PHASE_4_END = 110;

  private static double mAutonTimeLeft = -1;
  private static double mTeleopTimeLeft = -1;
  private static double mTimeLeftInPhase = -1;
  private static GameState mCurrentState = GameState.STANDBY;
  private static boolean mIsHubActive = false;
  private static Alliance mOurAlliance = null;
  private static Alliance mFirstActiveAlliance = null;

  public enum GameState {
    STANDBY,
    AUTON,
    TRANSITION,
    PHASE_1,
    PHASE_2,
    PHASE_3,
    PHASE_4,
    ENDGAME
  }

  public TransitionTracker() {}

  public boolean isHubActive() { return mIsHubActive; }
  public double getAutonTimeLeft() { return mAutonTimeLeft; }
  public double getTeleopTimeLeft() { return mTeleopTimeLeft; }
  public double getTimeLeftInPhase() { return mTimeLeftInPhase; }
  public GameState getCurrentState() { return mCurrentState; }

  // Called from robot.java
  public static void autonInit() {
    mOurAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    mFirstActiveAlliance = null;
    mAutonTimeLeft = Timer.getMatchTime();
    updateState(GameState.AUTON);
  }

  // Called from robot.java
  public static void teleopInit() {
    mOurAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    mAutonTimeLeft = -1;
    mTeleopTimeLeft = Timer.getMatchTime();
    updateState(GameState.TRANSITION);
  }

  public static void periodic() {
    if (mCurrentState == GameState.STANDBY) return;

    double matchTimeLeft = Timer.getMatchTime();
    if (matchTimeLeft < 0) {
      // No real match timing (not connected / no FMS time). Keep last known state.
      return;
    }

    if (DriverStation.isAutonomous()) {
      mAutonTimeLeft = matchTimeLeft;
      mTimeLeftInPhase = matchTimeLeft; // auton time remaining
      updateState(GameState.AUTON);
      return;
    }

    // Teleop / transition timing
    mTeleopTimeLeft = matchTimeLeft;
    double teleopTimeElapsed = TELEOP_TOTAL - matchTimeLeft;

    // Keep trying to get the first active alliance until we have it (covers "enabled late" cases too)
    if (mFirstActiveAlliance == null) {
      Alliance potentialAlliance = getFirstActiveAlliance();
      if (potentialAlliance != null) {
        mFirstActiveAlliance = potentialAlliance;
        recomputeHubActive(); // IMPORTANT: update hub-active even if state didn't change
      }
    }

    if (teleopTimeElapsed < TRANSITION_END) { // Transition
      updateState(GameState.TRANSITION);
      mTimeLeftInPhase = TRANSITION_END - teleopTimeElapsed;

    } else if (teleopTimeElapsed < PHASE_1_END) { // Phase 1
      updateState(GameState.PHASE_1);
      mTimeLeftInPhase = PHASE_1_END - teleopTimeElapsed;

    } else if (teleopTimeElapsed < PHASE_2_END) { // Phase 2
      updateState(GameState.PHASE_2);
      mTimeLeftInPhase = PHASE_2_END - teleopTimeElapsed;

    } else if (teleopTimeElapsed < PHASE_3_END) { // Phase 3
      updateState(GameState.PHASE_3);
      mTimeLeftInPhase = PHASE_3_END - teleopTimeElapsed;

    } else if (teleopTimeElapsed < PHASE_4_END) { // Phase 4
      updateState(GameState.PHASE_4);
      mTimeLeftInPhase = PHASE_4_END - teleopTimeElapsed;

    } else { // Endgame
      updateState(GameState.ENDGAME);
      mTimeLeftInPhase = matchTimeLeft; // time remaining in match
    }
  }

  private static void updateState(GameState pGameState) {
    if (pGameState == mCurrentState) return; // Only run on changes
    mCurrentState = pGameState;
    recomputeHubActive();
  }

  private static void recomputeHubActive() {
    if (mCurrentState == GameState.STANDBY) {
      mIsHubActive = false;
      return;
    }

    // If we don't know first-active yet (or alliance is unknown), pick a safe default.
    if (mOurAlliance == null || mFirstActiveAlliance == null) {
      mIsHubActive = true;
      return;
    }

    // If we're initially active, then odd phases, otherwise even phases
    if (mCurrentState == GameState.PHASE_1 || mCurrentState == GameState.PHASE_3) {
      mIsHubActive = mOurAlliance.equals(mFirstActiveAlliance);
    } else if (mCurrentState == GameState.PHASE_2 || mCurrentState == GameState.PHASE_4) {
      mIsHubActive = !mOurAlliance.equals(mFirstActiveAlliance);
    } else {
      mIsHubActive = true; // auton/transition/endgame
    }
  }

  private static Alliance getFirstActiveAlliance() {
    String gameData = DriverStation.getGameSpecificMessage();
    if(gameData.length() > 0){
      switch (Character.toUpperCase(gameData.charAt(0))){
        // Blue and red are fipped since gameData returns the first INACTIVE alliance, thus the other side must be active
        case 'B':
          return Alliance.Red;
        case 'R':
          return Alliance.Blue;
        default: // Data is screwed.
          Telemetry.log("<<< DATA IS CORRUPTED >>>");
      }
    } else {
      Telemetry.log("<<< UNSURE WHICH TEAM IS INACTIVE FIRST >>>");
    }

    return null;
  }
}
