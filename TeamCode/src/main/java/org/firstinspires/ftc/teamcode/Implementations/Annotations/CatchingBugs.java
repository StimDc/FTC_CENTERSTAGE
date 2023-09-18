package org.firstinspires.ftc.teamcode.Implementations.Annotations;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Method;
import java.util.ArrayList;

public class CatchingBugs {
    public static ArrayList<Class<?>> classes = new ArrayList<>();
    @Experimental
    public static void getExperimental(Telemetry telemetry,Class<?> clazz){
        Experimental annotation;
            for (Method method : clazz.getDeclaredMethods()) {
                if (method.isAnnotationPresent(Experimental.class)) {
                    telemetry.addLine("From class: " + clazz.getSimpleName() +", with method "+ method.getName());
                    telemetry.addLine("Marked as " + Experimental.class.getSimpleName());

                    annotation = method.getAnnotation(Experimental.class);
                    telemetry.addLine("Report: " + annotation.detail());
                }
            }

    }

    @Experimental
    public static String addToReport(String report, Object object, String detail){
        return report + "\n" + object.toString() + " " + detail;
    }
}
