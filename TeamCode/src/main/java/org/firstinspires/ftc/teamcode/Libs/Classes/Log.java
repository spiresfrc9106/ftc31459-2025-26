package org.firstinspires.ftc.teamcode.Libs.Classes;

import android.os.Environment;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

public class Log {
    private static final String baseFolder = "FIRST";

    public static void Write(String FileName, String Content){
        File file = new File(Environment.getExternalStorageDirectory(), baseFolder);
        if(!file.exists()){
            file.mkdir();
        }
        File file2 = new File(file, FileName);
        try {
            file2.createNewFile();
            Writer writer = new FileWriter(file2);
            writer.write(Content);
            writer.flush();
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


}
