// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team4201.utils;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/** Add your docs here. */
public class SendableEnumChooser<V extends Enum<V>> extends SendableChooser<V> {

    public SendableEnumChooser() {
        Class<V> enumClass = getEnumClass();
        V[] enumValues = enumClass.getEnumConstants();
        for (V enumValue : enumValues) {
            super.addOption(enumValue.name(), enumValue);
        }
    }

    // Helper method to retrieve the enum class
    @SuppressWarnings("unchecked")
    private Class<V> getEnumClass() {
        return (Class<V>) getSelected().getClass();
    }
}
