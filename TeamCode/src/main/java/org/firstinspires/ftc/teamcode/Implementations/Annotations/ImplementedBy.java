package org.firstinspires.ftc.teamcode.Implementations.Annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.SOURCE)
@Target(ElementType.METHOD)
public @interface ImplementedBy {
    public String name() default "";
    public String date() default "";
}
