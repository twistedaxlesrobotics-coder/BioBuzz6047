package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

/**
 * BiobuzzTagMap
 * Where each AprilTag is on the field (inches, field-center origin), learned by Camera Check and
 * saved in a small file on the Robot Controller so it is still there next time. The camera
 * correction only uses tags that are in here.
 *
 * Learn the positions with the hive in its normal (up) position and the robot at a spot you have
 * measured. If the field origin, the hive, or the aim points change, learn them again.
 */
public class BiobuzzTagMap {

    private static final String FILE_NAME = "biobuzz_tag_positions.txt";

    private static final Map<Integer, double[]> positions = new HashMap<>();
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
                String[] parts = line.trim().split(",");
                if (parts.length != 3) continue;
                try {
                    int id = Integer.parseInt(parts[0].trim());
                    double x = Double.parseDouble(parts[1].trim());
                    double y = Double.parseDouble(parts[2].trim());
                    positions.put(id, new double[] {x, y});
                } catch (NumberFormatException ignored) {
                    // skip a bad line
                }
            }
        } catch (Exception e) {
            // start empty if the file cannot be read
        }
    }

    public static synchronized boolean known(int id) {
        load();
        return positions.containsKey(id);
    }

    /** The tag's field X, or NaN if it has not been learned. */
    public static synchronized double x(int id) {
        load();
        double[] p = positions.get(id);
        return p == null ? Double.NaN : p[0];
    }

    /** The tag's field Y, or NaN if it has not been learned. */
    public static synchronized double y(int id) {
        load();
        double[] p = positions.get(id);
        return p == null ? Double.NaN : p[1];
    }

    public static synchronized int count() {
        load();
        return positions.size();
    }

    /** Remembers a tag position (replacing any old one). Call save() afterward to keep it. */
    public static synchronized void learn(int id, double x, double y) {
        load();
        positions.put(id, new double[] {x, y});
    }

    /** Forgets every learned tag. Call save() afterward to clear the saved file too. */
    public static synchronized void clear() {
        load();
        positions.clear();
    }

    /** Writes the positions to the file. Returns false if it could not be saved. */
    public static synchronized boolean save() {
        try {
            StringBuilder text = new StringBuilder();
            for (Map.Entry<Integer, double[]> entry : new TreeMap<>(positions).entrySet()) {
                text.append(entry.getKey()).append(',')
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
