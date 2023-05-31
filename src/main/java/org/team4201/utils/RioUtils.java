package org.team4201.utils;

public class RioUtils {
    enum SERIAL_NUMBER {
        SIMULATION("");
        private final String serialNumber;

        SERIAL_NUMBER(final String serialNumber) {
            this.serialNumber = serialNumber;
        }

        public String get() {
            return serialNumber;
        }
    }
}
