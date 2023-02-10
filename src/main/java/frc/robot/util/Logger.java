
package frc.robot.util;

import java.time.format.DateTimeFormatter;
import java.io.File;
import java.io.FileWriter;
import java.time.LocalDateTime;

public class Logger {
    
    DateTimeFormatter DTF = DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss");
    LocalDateTime time = LocalDateTime.now();

    String name = DTF.format(time);
    String filepath = "Users/9105/Desktop/" + name + ".csv";
    String titles;

    File file;
    FileWriter writer;
    
    public Logger() {
        try {
            file = new File(filepath);
            writer = new FileWriter(file);

            file.createNewFile();
            titles = "Left Temp" +"," + "Right Temp" + "," + "Velocoity" + "," + "Voltage";
            writer.write(titles);
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void logTelemetryData(double leftMotorTemp, double rightMotorTemp, double motorVelocity, double pdhVolts) {
        try {
            StringBuilder msgBuilder = new StringBuilder();
            msgBuilder.append(leftMotorTemp)
            .append(",")
            .append(rightMotorTemp)
            .append(",")
            .append(motorVelocity)
            .append(",")
            .append(pdhVolts);

            writer.write(msgBuilder.toString());
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }
}
