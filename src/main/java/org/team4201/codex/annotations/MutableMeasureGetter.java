package org.team4201.codex.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Target({ElementType.FIELD, ElementType.TYPE})
@Retention(RetentionPolicy.SOURCE)
public @interface MutableMeasureGetter {
  /**
   * The name of the function to generate.
   *
   * @return function name
   */
  String functionName();

  /**
   * The type of argument the function will accept.
   *
   * @return argument type as a Class
   */
  Class<?> argumentType();

  /**
   * The name of the parameter in the generated function.
   *
   * @return parameter name
   */
  String parameterName() default "arg";
}
