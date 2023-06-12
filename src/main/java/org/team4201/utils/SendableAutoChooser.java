// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team4201.utils;

import java.lang.reflect.Constructor;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

/** Add your docs here. */
public class SendableAutoChooser {
    private SendableChooser<Command> m_chooser = new SendableChooser<>();
    private List<Class<?>> m_classes;
    private List<Object> m_initedClasses;
    private List<String> m_classNames;

    public SendableAutoChooser(String packageName, Subsystem[] subsystems) {
        m_classes = getClasses(packageName);

        for (Class<?> clazz : m_classes) {
            try {
                Command obj = (Command) autoInitClassFromSubsystems(subsystems, clazz);
                m_initedClasses.add(obj);
                m_classNames.add(clazz.getName());
                m_chooser.addOption(clazz.getName(), obj);
            } catch (Exception e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    }

    public void setDefaultOption(String name) {
        try {
            m_chooser.setDefaultOption(name, getAutoCommandFromName(name));
        } catch (ClassNotFoundException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public SendableChooser<?> getChooser() {
        return m_chooser;
    }
    
    private Command getAutoCommandFromName(String name) throws ClassNotFoundException {
        for (String className : m_classNames) {
            if (className == name) {
                return (Command) m_initedClasses.get(m_classNames.indexOf(className));
            }
        }
        throw new ClassNotFoundException("Cannot find auto command");
    }

    // THIS WAS WRITTEN BY AI
    private static List<Class<?>> getClasses(String packageName) {
        List<Class<?>> classes = new ArrayList<>();

        try {
            ClassLoader classLoader = Thread.currentThread().getContextClassLoader();
            String path = packageName.replace('.', '/');
            Enumeration<URL> resources = classLoader.getResources(path);

            while (resources.hasMoreElements()) {
                URL resource = resources.nextElement();
                File file = new File(resource.getFile());

                if (file.isDirectory()) {
                    classes.addAll(findClasses(file, packageName));
                } else if (file.getName().endsWith(".class")) {
                    String className = packageName + '.' + file.getName().substring(0, file.getName().length() - 6);
                    Class<?> clazz = Class.forName(className);
                    classes.add(clazz);
                }
            }
        } catch (IOException | ClassNotFoundException e) {
            e.printStackTrace();
        }

        return classes;
    }

    // THIS WAS WRITTEN BY AI
    private static List<Class<?>> findClasses(File directory, String packageName) throws ClassNotFoundException {
        List<Class<?>> classes = new ArrayList<>();

        File[] files = directory.listFiles();
        if (files != null) {
            for (File file : files) {
                if (file.isDirectory()) {
                    assert !file.getName().contains(".");
                    classes.addAll(findClasses(file, packageName + "." + file.getName()));
                } else if (file.getName().endsWith(".class")) {
                    String className = packageName + '.' + file.getName().substring(0, file.getName().length() - 6);
                    Class<?> clazz = Class.forName(className);
                    classes.add(clazz);
                }
            }
        }

        return classes;
    }

    // THIS WAS WRITTEN BY AI
    private static Object autoInitClassFromSubsystems(Subsystem[] subsystems, Class<?> clazz) throws Exception {
        // Get the constructor of the class
        Constructor<?> constructor = getConstructorForSubsystems(clazz, subsystems);

        // Retrieve the parameter types
        Class<?>[] parameterTypes = constructor.getParameterTypes();

        // Match subsystem instances with parameter types
        Object[] subsystemInstances = new Object[parameterTypes.length];
        for (int i = 0; i < parameterTypes.length; i++) {
            Class<?> parameterType = parameterTypes[i];
            subsystemInstances[i] = findMatchingSubsystem(parameterType, subsystems);
        }

        // Create an instance of the class
        Object classInstance = constructor.newInstance(subsystemInstances);

        return classInstance;
    }

    // THIS WAS WRITTEN BY AI
    private static Constructor<?> getConstructorForSubsystems(Class<?> clazz, Subsystem[] subsystems) throws NoSuchMethodException {
        Class<?>[] subsystemClasses = new Class<?>[subsystems.length];
        for (int i = 0; i < subsystems.length; i++) {
            subsystemClasses[i] = subsystems[i].getClass();
        }
        return clazz.getDeclaredConstructor(subsystemClasses);
    }

    // THIS WAS WRITTEN BY AI
    private static Subsystem findMatchingSubsystem(Class<?> subsystemType, Subsystem[] subsystems) {
        for (Subsystem subsystem : subsystems) {
            if (subsystem.getClass().equals(subsystemType)) {
                return subsystem;
            }
        }
        return null; // Handle case when no matching subsystem is found
    }
}