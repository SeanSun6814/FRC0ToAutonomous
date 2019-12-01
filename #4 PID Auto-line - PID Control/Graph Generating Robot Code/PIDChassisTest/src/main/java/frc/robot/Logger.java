package frc.robot;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;

public class Logger {
    private PrintWriter writer = null;

    public Logger() {
        try {
            String time = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss").format(new Date());
            writer = new PrintWriter("/home/lvuser/Log " + time + ".csv");
            System.out.println("Log started");
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    public void log(Object message) {
        writer.print(message + ", ");
    }

    public void flush() {
        writer.flush();
        System.out.println("Log flushed");
    }
}