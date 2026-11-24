package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

/**
 * BiobuzzTagMap
 * Where each AprilTag, or each cluster of tags (a whole hive), is on the field (inches,
 * field-center origin), learned by Camera Check and saved in a small file on the Robot Controller so
 * it is still there next time. The camera correction only uses things that are in here.
 *
 * Each entry is named by its key: the tag number for a single tag (such as "34"), or the cluster
 * name the SDK gives a hive (such as "red audience").
 *
 * Learn the positions with the hive in its normal (up) position and the robot at a spot you know
 * exactly. If the field origin or the hive changes, learn them again.
 */
public class BiobuzzTagMap {

    private static final String FILE_NAME = "biobuzz_tag_positions.txt";

    private static final Map<String, double[]> positions = new HashMap<>();
    private static boolean loaded = false;

    private BiobuzzTagMap() {}

    /** Reads the saved positions the first time it is called. Safe to call again. */
    public static synchronized void load() {
        if (loaded) return;
        loaded = true;
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            if (!file.exists()) return;
            String text = ReadWriteFile.readFile(file);
            for (String line : text.split("\n")) {
                // The key may contain spaces but not commas: key,x,y
                int last = line.lastIndexOf(',');
                int prev = last > 0 ? line.lastIndexOf(',', last - 1) : -1;
                if (prev < 0 || last < 0) continue;
                try {
                    String key = line.substring(0, prev).trim();
                    double x = Double.parseDouble(line.substring(prev + 1, last).trim());
                    double y = Double.parseDouble(line.substring(last + 1).trim());
                    if (!key.isEmpty()) positions.put(key, new double[] {x, y});
                } catch (NumberFormatException ignored) {
                    // skip a bad line
                }
            }
        } catch (Exception e) {
            // start empty if the file cannot be read
        }
    }

    public static synchronized boolean known(String key) {
        load();
        return key != null && positions.containsKey(key);
    }

    /** The field X, or NaN if it has not been learned. */
    public static synchronized double x(String key) {
        load();
        double[] p = positions.get(key);
        return p == null ? Double.NaN : p[0];
    }

    /** The field Y, or NaN if it has not been learned. */
    public static synchronized double y(String key) {
        load();
        double[] p = positions.get(key);
        return p == null ? Double.NaN : p[1];
    }

    public static synchronized int count() {
        load();
        return positions.size();
    }

    /** Remembers a position (replacing any old one). Call save() afterward to keep it. */
    public static synchronized void learn(String key, double x, double y) {
        load();
        positions.put(key, new double[] {x, y});
    }

    /** Forgets everything learned. Call save() afterward to clear the saved file too. */
    public static synchronized void clear() {
        load();
        positions.clear();
    }

    /** Writes the positions to the file. Returns false if it could not be saved. */
    public static synchronized boolean save() {
        try {
            StringBuilder text = new StringBuilder();
            for (Map.Entry<String, double[]> entry : new TreeMap<>(positions).entrySet()) {
                text.append(entry.getKey().replace(',', ' ')).append(',')
                        .append(entry.getValue()[0]).append(',')
                        .append(entry.getValue()[1]).append('\n');
            }
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            ReadWriteFile.writeFile(file, text.toString());
            return true;
        } catch (Exception e) {
            return false;
        }
    }
}
