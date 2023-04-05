package frc.robot.util;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a helper class which wraps WPILib {@link Preferences}.  The intent is to provide an
 * abstracted way for a programmer to interact with Preferences and the SmartDashboard in tandem,
 * allowing the team to store values per robot and update those values live when testing to find
 * the appropriate final values.
 */
public class Settings {

  /**
   * Loads a boolean from the Preferences table (stored on the roboRIO and loaded as part of the Network Tables) and display it on the SmartDashboard for live edits
   *
   * @param subsystem the name of the subsystem to which the value belongs
   * @param attribute the specific attribute in the subsystem to which the value belongs
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the key (subsystem and attribute) or the given default value if there is no value
   *     associated with the key
   */
    public static boolean loadBoolean(String subsystem, String attribute, boolean defaultValue) {
        boolean value = Preferences.getBoolean(subsystem + "." + attribute, defaultValue);
        SmartDashboard.putBoolean(subsystem + "." + attribute, value);
        return value;
    }

  /**
   * Puts a boolean into the Preferences table, which will save it to the roboRIO
   *
   * @param subsystem the name of the subsystem to which the value belongs
   * @param attribute the specific attribute in the subsystem to which the value belongs
   * @param value the value to be saved
   */
    public static void saveBoolean(String subsystem, String attribute, boolean value) {
        Preferences.setBoolean(subsystem + "." + attribute, value);
    }

  /**
   * Returns a boolean value from a SmartDashboard entry
   *
   * @param subsystem the name of the subsystem to which the value belongs
   * @param attribute the specific attribute in the subsystem to which the value belongs
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the key (subsystem and attribute) or the given default value if there is no value
   *     associated with the key
   */
    public static boolean getLiveBoolean(String subsystem, String attribute, boolean defaultValue) {
        return SmartDashboard.getBoolean(subsystem + "." + attribute, defaultValue);
    }

  /**
   * Loads a double from the Preferences table (stored on the roboRIO and loaded as part of the Network Tables) and display it on the SmartDashboard for live edits
   *
   * @param subsystem the name of the subsystem to which the value belongs
   * @param attribute the specific attribute in the subsystem to which the value belongs
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the key (subsystem and attribute) or the given default value if there is no value
   *     associated with the key
   */
    public static double loadDouble(String subsystem, String attribute, double defaultValue) {
        double value = Preferences.getDouble(subsystem + "." + attribute, defaultValue);
        SmartDashboard.putNumber(subsystem + "." + attribute, value);
        return value;
    }

  /**
   * Puts a double into the Preferences table, which will save it to the roboRIO
   *
   * @param subsystem the name of the subsystem to which the value belongs
   * @param attribute the specific attribute in the subsystem to which the value belongs
   * @param value the value to be saved
   */
    public static void saveDouble(String subsystem, String attribute, double value) {
        Preferences.setDouble(subsystem + "." + attribute, value);
    }

  /**
   * Returns a double value from a SmartDashboard entry
   *
   * @param subsystem the name of the subsystem to which the value belongs
   * @param attribute the specific attribute in the subsystem to which the value belongs
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the key (subsystem and attribute) or the given default value if there is no value
   *     associated with the key
   */
    public static double getLiveDouble(String subsystem, String attribute, double defaultValue) {
        return SmartDashboard.getNumber(subsystem + "." + attribute, defaultValue);
    }

  /**
   * Loads a String from the Preferences table (stored on the roboRIO and loaded as part of the Network Tables) and display it on the SmartDashboard for live edits
   *
   * @param subsystem the name of the subsystem to which the value belongs
   * @param attribute the specific attribute in the subsystem to which the value belongs
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the key (subsystem and attribute) or the given default value if there is no value
   *     associated with the key
   */
    public static String loadString(String subsystem, String attribute, String defaultValue) {
        String value = Preferences.getString(subsystem + "." + attribute, defaultValue);
        SmartDashboard.putString(subsystem + "." + attribute, value);
        return value;
    }

  /**
   * Puts a String into the Preferences table, which will save it to the roboRIO
   *
   * @param subsystem the name of the subsystem to which the value belongs
   * @param attribute the specific attribute in the subsystem to which the value belongs
   * @param value the value to be saved
   */
    public static void saveString(String subsystem, String attribute, String value) {
        Preferences.setString(subsystem + "." + attribute, value);
    }

  /**
   * Returns a String value from a SmartDashboard entry
   *
   * @param subsystem the name of the subsystem to which the value belongs
   * @param attribute the specific attribute in the subsystem to which the value belongs
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the key (subsystem and attribute) or the given default value if there is no value
   *     associated with the key
   */
    public static String getLiveString(String subsystem, String attribute, String defaultValue) {
        return SmartDashboard.getString(subsystem + "." + attribute, defaultValue);
    }
}
