package frc.utility.json;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;

import frc.auton.guiauto.serialization.Autonomous;

import java.io.File;
import java.io.IOException;

public class Serializer {
    static ObjectMapper objectMapper = new ObjectMapper();

    public static String serializeToString(Object obj) throws IOException {
        objectMapper.configure(SerializationFeature.INDENT_OUTPUT, false);
        return objectMapper.writeValueAsString(obj);
    }

    public static void serializeToFile(Object obj, File file) throws IOException {
        objectMapper.configure(SerializationFeature.INDENT_OUTPUT, false);
        objectMapper.writeValue(file, obj);
    }

    public static Autonomous deserializeAutoFromFile(File file) throws IOException {
        return objectMapper.readValue(file, Autonomous.class);
    }

    public static Object deserialize(String object, Class<?> serializableObject) throws IOException {
        return objectMapper.readValue(object, serializableObject);
    }

    public static Autonomous deserializeAuto(String object) throws IOException, ClassNotFoundException {
        return objectMapper.readValue(object, Autonomous.class);
    }

    public static Object deserializeFromFile(File file, Class<?> serializableObject) throws IOException {
        return objectMapper.readValue(file, serializableObject);
    }

}
