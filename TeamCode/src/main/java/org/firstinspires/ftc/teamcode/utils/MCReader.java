package org.firstinspires.ftc.teamcode.utils;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;

public class MCReader {

    public int[] MCsheet;
    public int Size;

    public void Init(String csvFile, int Size) {
        this.Size = Size;
        MCsheet = new int[Size];
        MCsheet = readCSV(csvFile);
    }

    public static int[] readCSV(String csvFile) {
        int[] values = new int[63];

        //Check if the file location is valid
        values[0] = 200; // Debugging value
        File file = new File(csvFile);
        if (file.exists()) {
            values[1] = 200; // Debugging value
        } else {
            values[2] = 200; // Debugging value
            // If the file doesn't exist, set all the values to 99
            for (int i = 20; i < 63; i++) {
                values[i] = 99;
            }
            return values;
        }


        values[3] = 200; // Debugging value
        try (BufferedReader br = new BufferedReader(new FileReader(csvFile))) {
            values[4] = 200; // Debugging value
            String line = br.readLine();
            values[5] = 200; // Debugging value
            if (line != null) {
                values[6] = 200; // Debugging value
                String[] columns = line.split(",");
                values[7] = 200; // Debugging value
                // Ensure we don't go out of bounds (up to the first 63 columns)
                int numColumnsToRead = Math.min(63, columns.length);
                values[8] = 200; // Debugging value
                // Convert and copy the values from the CSV into the array
                for (int i = 0; i < numColumnsToRead; i++) {
                    // Check if the section is blank, and set it to 99
                    if (columns[i].trim().isEmpty()) {
                        values[i] = 99;
                    } else {
                        values[i] = Integer.parseInt(columns[i]);
                    }
                }
            }
        } catch (IOException | NumberFormatException e) {
            e.printStackTrace();
            // If there's an error, set all the values to 99
            for (int i = 20; i < 63; i++) {
                values[i] = 99;
            }
        }

        return values;
    }
}
