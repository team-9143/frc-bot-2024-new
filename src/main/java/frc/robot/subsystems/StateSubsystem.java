package frc.robot.subsystems;

// TODO(user): Remove if unnecessary
/**
 * Subsystem with state-based control and a stop method for safety.
 *
 * @param <T> Enum containing control states of the subsystem
 */
public abstract class StateSubsystem<T extends Enum<T>> extends SafeSubsystem {
  /** Current state of the subsystem. */
  private T currentState;

  /** @param state new state of the subsystem */
  public void setState(T state) {
    currentState = state;
  }

  /** @return current state of the subsystem */
  public T getState() {
    return currentState;
  }
}