
package frc.robot.util;

import java.time.format.DateTimeFormatter;
import java.io.File;
import java.io.FileWriter;
import java.time.LocalDateTime;

public class CustomLogger {
    
    private DateTimeFormatter DTF = DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss");
    private LocalDateTime time = LocalDateTime.now();

    private String name = DTF.format(time);
    private String filepath = "/U/logs/" + name + ".csv";
    private String titles;

    private File file;
    private FileWriter writer;
    
    public CustomLogger() {
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
