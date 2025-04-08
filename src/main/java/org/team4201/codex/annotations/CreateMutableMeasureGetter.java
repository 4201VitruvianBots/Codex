package org.team4201.codex.annotations;

import javax.annotation.processing.*;
//
//
// public class CreateMutableMeasureGetter extends AbstractProcessor {
//        private Types typeUtils;
//        private Elements elementUtils;
//        private Filer filer;
//        private Messager messager;
//
//        @Override
//        public synchronized void init(ProcessingEnvironment processingEnv) {
//            super.init(processingEnv);
//            typeUtils = processingEnv.getTypeUtils();
//            elementUtils = processingEnv.getElementUtils();
//            filer = processingEnv.getFiler();
//            messager = processingEnv.getMessager();
//        }
//
//        @Override
//        public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv)
// {
//            for (Element element : roundEnv.getElementsAnnotatedWith(MutableMeasureGetter.class))
// {
//                if (element.getKind() != ElementKind.CLASS) {
//                    messager.printMessage(Diagnostic.Kind.ERROR,
//                            "@Processor can only be applied to classes", element);
//                    continue;
//                }
//
//                TypeElement classElement = (TypeElement) element;
//                MutableMeasureGetter annotation =
// classElement.getAnnotation(MutableMeasureGetter.class);
//
//                String packageName =
// elementUtils.getPackageOf(classElement).getQualifiedName().toString();
//                String className = classElement.getSimpleName().toString();
//                String processorClassName = className + "Processor";
//
//                try {
//                    createProcessorClass(packageName, className, processorClassName, annotation);
//                } catch (IOException e) {
//                    messager.printMessage(Diagnostic.Kind.ERROR,
//                            "Error creating processor class: " + e.getMessage(), element);
//                }
//            }
//            return true;
//        }
//
//        private void createProcessorClass(String packageName, String className,
//                String functionGetterClassName, MutableMeasureGetter annotation) throws
// IOException {
//            String functionName = annotation.functionName();
//            String argType = annotation.argumentType().getCanonicalName();
//            String paramName = annotation.parameterName();
//            boolean returnThis = annotation.returnThis();
//
//            String returnType = returnThis ? className : "void";
//            String returnStatement = returnThis ? "return object;" : "";
//
//            JavaFileObject javaFile = filer.createSourceFile(packageName + "." +
// functionGetterClassName);
//            try (PrintWriter out = new PrintWriter(javaFile.openWriter())) {
//                // Write the generated class
//                out.printf("""
//                        package %s;
//                        /**
//                         * Auto-generated FunctionGetter for %s
//                         */
//
//
//                        public class %s {");
//                            /**"
//                             * Get the object using the specified function
//                             * @param object The object to use
//                             * @param functionName The function to use
//                             * @param
//                             * @param " + paramName + " The argument to pass to the processing
// function");
//                             * @return
//
//                        """,
//                        packageName,
//                        className,
//                        functionGetterClassName
//                        );
//                out.println("
//                out.println("
//                out.println("
//                out.println("
//                out.println("
//                out.println("     */");
//                out.println("    public static " + returnType + " " + "get" +
// StringUtils.capitalize(functionName) + "() {");
//                out.println("        " + returnStatement);
//                out.println("    }");
//                out.println("}");
//            }
//        }
// }
