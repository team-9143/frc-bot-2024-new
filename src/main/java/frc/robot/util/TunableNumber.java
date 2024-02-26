package frc.robot.util;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleConsumer;
import frc.robot.logger.Logger;

import java.util.ArrayList;
import java.util.HashSet;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;

/** Represents a double that can be changed during runtime. */
public class TunableNumber implements DoubleSupplier {
  private static final String LOG_DIR = "/numbers/";

  /** List of instances. */
  private static final ArrayList<TunableNumber> s_instances = new ArrayList<TunableNumber>();
  /** List of groups. */
  private static final HashSet<String> s_groups = new HashSet<String>();

  /** Descriptor of the TunableNumber for use with dashboards. */
  public final String m_name;
  /** Descriptor of associated TunableNumbers for use with dashboards. */
  public final String m_group;

  /** Value of the TunableNumber at startup. */
  public final double m_default;
  /** Value of the TunableNumber. */
  private double m_value;

  /** NetworkTables entry of the number. */
  private GenericEntry m_entry;

  /** True if the value of the number can be changed. */
  private boolean m_mutable = false;

  /** Function to run when the value is changed. */
  private DoubleConsumer m_onChange;

  /**
   * Create a new TunableNumber.
   *
   * @param name descriptor for dashboards
   * @param val initial value
   * @param group descriptor for associated TunableNumbers
   */
  public TunableNumber(String name, double val, String group) {
    m_name = name;
    m_default = val;
    m_value = val;
    m_group = group;

    m_onChange = n -> Logger.recordOutput(LOG_DIR + m_group + "/" + m_name, n);
    m_onChange.accept(val);

    s_instances.add(this);
    s_groups.add(group);
  }

  /**
   * Create a new TunableNumber in the {@code default} group.
   *
   * @param name descriptor for dashboards
   * @param val initial value
   */
  public TunableNumber(String name, double val) {
    this(name, val, "default");
  }

  /** @return an array of all instances for class-wide changes (e.g. making all numbers mutable) */
  public static TunableNumber[] getAllInstances() {
    return s_instances.toArray(new TunableNumber[s_instances.size()]);
  }

  /** @return an array of all TunableNumber groups */
  public static String[] getAllGroups() {
    return s_groups.toArray(new String[s_groups.size()]);
  }

  /**
   * Get all instances within a specified group.
   *
   * @param group group of instances to return (case sensitive)
   * @return an array of all instances in the group
   */
  public static TunableNumber[] getGroup(String group) {
    final ArrayList<TunableNumber> fullGroup = new ArrayList<TunableNumber>();

    fullGroup.addAll(s_instances);

    fullGroup.removeIf(e -> !e.m_group.equals(group));
    return fullGroup.toArray(new TunableNumber[fullGroup.size()]);
  }

  /** Update all numbers with values from NetworkTables, if provided. */
  public static void updateAll() {
    s_instances.forEach(e -> e.update());
  }

  /** Update number with value from NetworkTables entry, if provided. */
  public void update() {
    if (m_entry != null) {
      set(m_entry.getDouble(m_value));
    }
  }

  /** @return {@code true} if the number is mutable */
  public boolean getMutable() {
    return m_mutable;
  }

  /** @param mutable {@code true} if the number should be mutable */
  public void setMutable(boolean mutable) {
    m_mutable = mutable;
  }

  /**
   * Adds a consumer to be called when the value is changed.
   *
   * @param onChange {@link DoubleConsumer} to be called with the new value
   */
  public void bind(DoubleConsumer onChange) {
    m_onChange = m_onChange.andThen(onChange);
  }

  /** Resets value to initialized value. */
  public void reset() {
    set(m_default);
  }

  /** @return the current value */
  @Override
  public double getAsDouble() {
    return m_value;
  }

  /**
   * Change the value, ignoring duplicates. Only works if the number is mutable.
   *
   * @param val new value
   *
   * @see TunableNumber#getMutable
   * @see TunableNumber#setMutable
   */
  public void set(double val) {
    if (m_mutable && m_value != val) {
      m_value = val;
      if (m_entry != null) {
        m_entry.setDouble(val);
      }
      m_onChange.accept(m_value);
    }
  }

  /** Initalize unadded TunableNumbers to Shuffleboard and make them mutable. */
  public static void initializeShuffleboard() {
    var groups = getAllGroups();

    for (var group : groups) {
      var instances = TunableNumber.getGroup(group);
      var layout = Shuffleboard.getTab("Tunables").getLayout(group, BuiltInLayouts.kList).withSize(2, 6);

      for (var elem : instances) {
        // Add tunables to shuffleboard if not already added
        if (elem.m_entry == null) {
          // Make unadded tunables mutable
          elem.setMutable(true);

          elem.m_entry = layout.add(elem.m_name, elem.getAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(2, 2)
            .getEntry();
        }
      }
    }
  }
}