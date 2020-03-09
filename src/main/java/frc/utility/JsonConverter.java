package frc.utility;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.github.cliftonlabs.json_simple.JsonObject;
import java.util.Map;

public class JsonConverter {

    public Map<String, String> getJsonObject(String json) throws JsonMappingException, JsonProcessingException {
        /*JsonObject jsonObj = new JsonObject();
        JsonObject jsonHeader = new JsonObject();
        JsonObject jsonData = new JsonObject();

        String[] jsonChunks = json.split(",");
        String[] data;
        String type;
        //type = jsonChunks[0].split("type:\"")[1].split("\"}")[0];

        //jsonHeader.put(type, value)*/
        

        //return jsonObj;

        /*JsonObject[] jsonObjs;
        String[] jsonChunks = json.split("\"");

        for(int i = 0;i<jsonChunks.length;i++){
          //  if()
        }*/
        ObjectMapper mapper = new ObjectMapper();

        Map<String, String> map = mapper.readValue(json, Map.class);
        return map;
    }

}