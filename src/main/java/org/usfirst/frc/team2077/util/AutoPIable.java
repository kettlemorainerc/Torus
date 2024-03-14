package org.usfirst.frc.team2077.util;

import edu.wpi.first.wpilibj.Preferences;
import org.usfirst.frc.team2077.command.AutoPITuner;

public abstract class AutoPIable {

    private String saveKey;

    public void init(String saveKey, double defaultP, double defaultI, boolean override){
        this.saveKey = saveKey;

        Preferences.initDouble(saveKey + "_P", defaultP);
        Preferences.initDouble(saveKey + "_I", defaultI);

        if(!override){
            setP(getSavedP());
            setI(getSavedI());
        }else{
            Preferences.setDouble(saveKey + "_P", defaultP);
            Preferences.setDouble(saveKey + "_I", defaultP);

            setP(defaultP);
            setI(defaultI);
        }
    }

    public abstract double getP();
    public abstract double getI();

    public abstract void setP(double p);
    public abstract void setI(double i);

    public double getSavedP(){
        return Preferences.getDouble(saveKey + "_P", 0.0);
    }

    public double getSavedI(){
        return Preferences.getDouble(saveKey + "_I", 0.0);
    }

    public abstract double tunerGet();
    public abstract void tunerSet(double setpoint);

    public void savePI(){
        Preferences.setDouble(saveKey + "_P", getP());
        Preferences.setDouble(saveKey + "_I", getI());

        System.out.println(saveKey + "_P " + getP());
        System.out.println(saveKey + "_I " + getI());

    }

    public abstract AutoPITuner.ErrorMethod getErrorMethod();

}
