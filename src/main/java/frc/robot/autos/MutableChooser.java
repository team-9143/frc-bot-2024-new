package frc.robot.autos;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.util.sendable.SendableRegistry;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiConsumer;

/**
 * A {@link edu.wpi.first.wpilibj.smartdashboard.SendableChooser SendableChooser}-like class
 * allowing for the editing of options. Use {@link MutableChooser#isUpdateReq()} to check if an
 * update is necessary on shuffleboard by selecting the default option.
 */
public class MutableChooser<T extends Enum<T>> implements NTSendable, AutoCloseable {

  /** The key for the default value. */
  private static final String DEFAULT = "default";

  /** The key for the selected option. */
  private static final String SELECTED = "selected";

  /** The key for the active option. */
  private static final String ACTIVE = "active";

  /** The key for the option array. */
  private static final String OPTIONS = "options";

  /** The key for the instance number. */
  private static final String INSTANCE = ".instance";

  /**
   * A Lock to stop simultaneous editing of shuffleboard options, selection, or selection
   * publishers.
   */
  private final ReentrantLock m_networkLock = new ReentrantLock(true);

  /** A map linking options to their identifiers, for use with shuffleboard. */
  private final LinkedHashMap<String, T> m_linkedOptions = new LinkedHashMap<>();

  /** Key for the default selection. */
  private final String m_defaultKey;

  /** Default selection. */
  private final T m_defaultObj;

  /** Key for the current selection. */
  private String m_selectedKey;

  /** A Lock to stop simulataneous reading and writing to list of updates. */
  private final ReentrantLock m_updateLock = new ReentrantLock(true);

  /** A Set storing the options to be updated on the next update. */
  private final EnumSet<T> m_optionsWanted;

  /** If an update is required to sync options on shuffleboard. */
  private boolean m_updateReq = false;

  /** A consumer to be called with the old and new selections when the selection changes. */
  private BiConsumer<T, T> m_bindTo = null;

  /** ArrayList to keep track of publishers. */
  private final ArrayList<StringPublisher> m_activePubs = new ArrayList<>();

  /** Number of instances. */
  private final int m_instance;

  private static int s_instances = 0;

  /**
   * Instantiates a new Chooser with a default option.
   *
   * @param obj the default option
   */
  MutableChooser(T obj) {
    m_instance = s_instances++;
    SendableRegistry.add(this, "SendableChooser", m_instance);

    m_defaultKey = obj.toString();
    m_defaultObj = obj;
    m_selectedKey = m_defaultKey;

    m_optionsWanted = EnumSet.noneOf(obj.getDeclaringClass());
    m_linkedOptions.put(m_defaultKey, obj);
  }

  /** Syncs options on shuffleboard if default option is selected. */
  private void updateOptions() {
    m_networkLock.lock();
    m_updateLock.lock();
    try {
      if (!m_selectedKey.equals(m_defaultKey)) {
        return;
      }

      m_linkedOptions.clear();
      m_linkedOptions.put(m_defaultKey, m_defaultObj);
      m_optionsWanted.forEach(e -> m_linkedOptions.put(e.toString(), e));
      m_updateReq = false;
    } finally {
      m_networkLock.unlock();
      m_updateLock.unlock();
    }
  }

  /**
   * Adds an option to the chooser. Takes affect when the selected option is the default. Attempting
   * to add duplicate options will do nothing.
   *
   * @param option the option to add
   */
  public void add(T option) {
    m_updateLock.lock();
    try {
      if (m_optionsWanted.add(option)) {
        m_updateReq = true;
        updateOptions();
      }
    } finally {
      m_updateLock.unlock();
    }
  }

  /**
   * Removes an option from the chooser. Takes affect when the selected option is the default.
   * Attempting to remove the default option will do nothing.
   *
   * @param option the option to remove
   */
  public void remove(T option) {
    m_updateLock.lock();
    try {
      if (m_optionsWanted.remove(option)) {
        m_updateReq = true;
        updateOptions();
      }
    } finally {
      m_updateLock.unlock();
    }
  }

  /**
   * Sets all options in the chooser (not including the default). Takes affect when the selected
   * option is the default.
   *
   * @param options the options to be presented
   */
  @SafeVarargs
  public final void setAll(T... options) {
    List<T> optionList = List.of(options);

    m_updateLock.lock();
    try {
      if (m_optionsWanted.retainAll(optionList) || m_optionsWanted.addAll(optionList)) {
        m_updateReq = true;
        updateOptions();
      }
    } finally {
      m_updateLock.unlock();
    }
  }

  /**
   * If the chooser needs to be updated to sync with shuffleboard. Updates can be performed by
   * selecting the default option on shuffleboard.
   *
   * @return {@code true} if the chooser needs to be updated
   */
  public boolean isUpdateReq() {
    m_updateLock.lock();
    try {
      return m_updateReq;
    } finally {
      m_updateLock.unlock();
    }
  }

  /** Returns the selected option, and the default if there is no selection. */
  public T getSelected() {
    m_networkLock.lock();
    try {
      return m_linkedOptions.get(m_selectedKey);
    } finally {
      m_networkLock.unlock();
    }
  }

  /**
   * Binds a {@link BiConsumer} to a change in the chooser's selection. The first input is the old
   * option, and the second input is the new option.
   *
   * @param bindTo the consumer to bind to
   */
  public void bindTo(BiConsumer<T, T> bindTo) {
    m_bindTo = bindTo;
  }

  @Override
  public void initSendable(NTSendableBuilder builder) {
    builder.setSmartDashboardType("String Chooser");

    IntegerPublisher instancePub = new IntegerTopic(builder.getTopic(INSTANCE)).publish();
    instancePub.set(m_instance);
    builder.addCloseable(instancePub);

    builder.addStringProperty(DEFAULT, () -> m_defaultKey, null);
    builder.addStringArrayProperty(
        OPTIONS, () -> m_linkedOptions.keySet().toArray(new String[0]), null);

    builder.addStringProperty(
        ACTIVE,
        () -> {
          m_networkLock.lock();
          try {
            return m_selectedKey;
          } finally {
            m_networkLock.unlock();
          }
        },
        null);

    m_networkLock.lock();
    try {
      m_activePubs.add(new StringTopic(builder.getTopic(ACTIVE)).publish());
    } finally {
      m_networkLock.unlock();
    }

    builder.addStringProperty(
        SELECTED,
        null,
        newSelectionKey -> {
          T oldSelection, newSelection;

          m_networkLock.lock();
          try {
            oldSelection = m_linkedOptions.get(m_selectedKey);
            newSelection = m_linkedOptions.get(newSelectionKey);

            m_activePubs.forEach(pub -> pub.set(newSelectionKey));
            m_selectedKey = newSelectionKey;

            if (m_updateReq) {
              updateOptions();
            }
          } finally {
            m_networkLock.unlock();
          }

          if (m_bindTo != null) {
            m_bindTo.accept(oldSelection, newSelection);
          }
        });
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
    m_networkLock.lock();
    try {
      m_activePubs.forEach(pub -> pub.close());
    } finally {
      m_networkLock.unlock();
    }
  }
}
