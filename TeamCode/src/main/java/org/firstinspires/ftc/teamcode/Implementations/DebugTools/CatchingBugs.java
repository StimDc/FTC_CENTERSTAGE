package org.firstinspires.ftc.teamcode.Implementations.DebugTools;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ControlPart.Sponsori;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.Experimental;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.ImplementedBy;

import java.lang.reflect.Method;
import java.util.ArrayList;

public class CatchingBugs {
    private static final Class<?> classes[] ={Sponsori.class, CatchingBugs.class, ComputerTool.class};

    /**
     * Prints the methods marked with the @Experimental annotation on the phone
     * @param telemetry device linked with the phone
     * @param clazz the class where the methods are taken from
     */
    @ImplementedBy(name="Andrei", date="19.09.23")
    @Experimental
    public static void getExperimental(Telemetry telemetry,Class<?> clazz){
        for (Method method : clazz.getDeclaredMethods()) {
            if (method.isAnnotationPresent(Experimental.class)) {
                telemetry.addLine( clazz.getSimpleName() +" : " + method.getName());
                }
            }

    }

    /**
     * Prints the methods marked with the @Experimental annotation on PC
     * @param clazz the class where the methods are taken from
     * @param report
     */
    @Experimental
    public static void getExperimental(Class<?> clazz,ReportBuilder report){
        for (Method method : clazz.getDeclaredMethods()) {
            Experimental annotation;
            if (method.isAnnotationPresent(Experimental.class)) {
                System.out.println("From class: " + clazz.getSimpleName() +", with method "+ method.getName());
                System.out.println("Marked as " + Experimental.class.getSimpleName());

                annotation = method.getAnnotation(Experimental.class);
                System.out.println("Report : " + annotation.detail());

            }
        }

    }


    @Experimental
    public static void getNameReport(Telemetry telemetry, String name){
        ImplementedBy annotation;
        for(Class<?> clazz: classes){
            for(Method method: clazz.getDeclaredMethods()){
                telemetry.addLine(name + " worked on:");
                if(method.isAnnotationPresent(ImplementedBy.class)){
                    annotation = method.getAnnotation(ImplementedBy.class);
                    if(annotation.name().equals( name)){
                        telemetry.addLine(method.getName());
                    }

                }
            }
        }

    }
    @Experimental
    public static void getNameReport(String name){
        ImplementedBy annotation;
        System.out.println(name + " worked on:");
        for(Class<?> clazz : classes) {
            for (Method method : clazz.getDeclaredMethods()) {

                if (method.isAnnotationPresent(ImplementedBy.class)) {

                    annotation = method.getAnnotation(ImplementedBy.class);
                    if (annotation.name().equals(name)) {
                        System.out.println(clazz.getSimpleName() + " : " + method.getName());
                    }

                }
            }
        }

    }
    @Experimental
    public static String addToReport(String report, Object object, String detail){
        return report + "\n" + object.toString() + " " + detail;
    }
}
