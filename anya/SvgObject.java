package pgraph;

import java.util.HashMap;
public class SvgObject
{
    String svgType;
    HashMap<String,String>[] objects;
    String[] variableNames;

    public SvgObject(HashMap<String,String>[] svgObjects, String[] variableNames){

        for(HashMap<String,String> attr : svgObjects)
        {
            if(attr.get("tag") == null){ throw new IllegalArgumentException("No svg tag");}
        }

        this.objects = svgObjects;
        this.variableNames = variableNames;
    }

    public int getArgumentLength(){return variableNames.length;}

    public String getOutputString(){
        String output = "{ \"attributes\": [";
        for(HashMap<String,String> attributes : objects)
        {
            if(attributes != objects[0]){output+=',';}
            output += "[";
            for(String key : attributes.keySet()) {
                if(output.charAt(output.length()-1) != '['){output+=',';}
                output += "{\"key\":\""+key+"\",\"value\":\""+attributes.get(key)+"\"}";

            }
            output += "]";
        }
        output += "], \"variableNames\": [";
        for(int i = 0; i < variableNames.length; i++)
        {
            output +=  "\""+variableNames[i]+"\"" ;
            if(i != variableNames.length -1) {
                output += ",";
            }
        }
        output += "]}";
        return output;
    }
}