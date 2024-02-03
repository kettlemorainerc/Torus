package org.usfirst.frc.team2077.util;

import edu.wpi.first.wpilibj.Preferences;

public abstract class AutoPIable {

    private String saveKey;

    AutoPIable(String saveKey, double defaultP, double defaultI){
        this.saveKey = saveKey;
        Preferences.initDouble(String.format("%s-P", saveKey), defaultP);
        Preferences.initDouble(String.format("%s-I", saveKey), defaultI);
    }

    AutoPIable(String saveKey){
        this(saveKey, 0.0, 0.0);
    }

    abstract double getP();
    abstract double getI();

    abstract void setP();
    abstract void setI();

    double getSavedP(){
        return Preferences.getDouble(String.format("%s-P", saveKey), 0.0);
    }

    double getSavedI(){
        return Preferences.getDouble(String.format("%s-I", saveKey), 0.0);
    }

    void savePI(){
        Preferences.setDouble(String.format("%s-P", saveKey), getP());
        Preferences.setDouble(String.format("%s-I", saveKey), getI());
    }

}
