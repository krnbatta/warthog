package pgraph;
import pgraph.EventType;
import pgraph.anya.AnyaNode;

import java.io.*;
import java.util.*;

public class Debug {

  String outputString;
  SvgObject svgObject;

  String eventString;

  String nodeString;

  HashMap<String, HashMap<String,String>> nodeVariables;
  HashMap<String,String> parentNodes;
  HashMap<String,double[]> nodeValues;

  public Debug(){
    outputString = "{";
    nodeString = "\"nodeStructure\": [{\"type\": \"circle\", \"variables\": {\"cx\": \"cx\", \"cy\": \"cy\"}, \"persisted\": true, \"drawPath\": true},{\"type\": \"line\", \"variables\": {\"x1\": \"x1\", \"y1\": \"y1\", \"x2\": \"x2\", \"y2\": \"y2\"}, \"persisted\": false},{\"type\": \"line\", \"variables\": {\"x1\": \"cx\", \"y1\": \"cy\", \"x2\": \"x1\", \"y2\": \"y1\"}, \"persisted\": false},{\"type\": \"line\", \"variables\": {\"x1\": \"cx\", \"y1\": \"cy\", \"x2\": \"x2\", \"y2\": \"y2\"}, \"persisted\": false},{\"type\": \"polygon\", \"variables\": {\"points\": [\"x1\", \"y1\", \"x2\", \"y2\", \"cx\", \"cy\"]}, \"persisted\": false}]";
    eventString = "\"eventList\":[";
    nodeVariables = new HashMap<String, HashMap<String,String>>();
    parentNodes = new HashMap<String, String>();
    nodeValues = new HashMap<String, double[]>();
  }

  public String addStartNode(AnyaNode start){
    String cx = Double.toString(Math.round(start.root.x*10.0)/10.0);
    String cy = Double.toString(Math.round(start.root.y*10.0)/10.0);
    String x1 = Double.toString(Math.round(start.root.x*10.0)/10.0);
    String x2 = Double.toString(Math.round(start.root.x*10.0)/10.0);
    String y1 = Double.toString(Math.round(start.root.y*10.0)/10.0);
    String y2 = Double.toString(Math.round(start.root.y*10.0)/10.0);
    return "{\"type\": \"source\", \"id\":" + Integer.toString(start.getId()) + ", \"variables\": {" +
    "\"cx\": " + cx + ", \"cy\": " + cy + ", " +
    "\"x1\": " + x1 + ", \"y1\": " + y1 + ", " +
    "\"x2\": " + x2 + ", \"y2\": " + y2 + "}}";
  }

  public String addTargetNode(AnyaNode target){
    String cx = Double.toString(Math.round(target.root.x*10.0)/10.0);
    String cy = Double.toString(Math.round(target.root.y*10.0)/10.0);
    String x1 = Double.toString(Math.round(target.root.x*10.0)/10.0);
    String x2 = Double.toString(Math.round(target.root.x*10.0)/10.0);
    String y1 = Double.toString(Math.round(target.root.y*10.0)/10.0);
    String y2 = Double.toString(Math.round(target.root.y*10.0)/10.0);
    return "{\"type\": \"destination\", \"id\":" + Integer.toString(target.getId()) + ", \"variables\": {" +
    "\"cx\": " + cx + ", \"cy\": " + cy + ", " +
    "\"x1\": " + x1 + ", \"y1\": " + y1 + ", " +
    "\"x2\": " + x2 + ", \"y2\": " + y2 + "}}";
  }

  public void startEvent(AnyaNode start, AnyaNode target){
    if(eventString.length() > 15) {
      eventString += ",\n";
    }
    eventString += addStartNode(start) + ", " + addTargetNode(target) + ", ";
  }

  public void endEvent(AnyaNode end, int parentId){
    HashMap<String, String> variables = nodeVariables.get(end.getId());
    String cx = Double.toString(Math.round(end.root.x*10.0)/10.0);
    String cy = Double.toString(Math.round(end.root.y*10.0)/10.0);
    String x1 = Double.toString(Math.round(end.root.x*10.0)/10.0);
    String x2 = Double.toString(Math.round(end.root.x*10.0)/10.0);
    String y1 = Double.toString(Math.round(end.root.y*10.0)/10.0);
    String y2 = Double.toString(Math.round(end.root.y*10.0)/10.0);
    eventString += "{\"type\": \"" + EventType.FINISH.toString() + "\", \"id\":" + Integer.toString(end.getId()) + ", \"variables\": {" +
    "\"cx\": " + cx + ", \"cy\": " + cy + ", " +
    "\"x1\": " + x1 + ", \"y1\": " + y1 + ", " +
    "\"x2\": " + x2 + ", \"y2\": " + y2 + "}, " +
    "\"f\": \"" + Double.toString(end.getF()) + "\", " +
    "\"g\":\"" + Double.toString(end.getG()) + "\", " +
    "\"pId\":" + Integer.toString(parentId) +
    "}";
  }

  public String getEventString(String eventType, String id, String parentId, HashMap<String,String> variables,double g, double f){
    String eventStr = "";
    String pId = parentId != null ? "\""+parentId+"\"" : null;
    eventStr += "{\"id\":\""+id+"\",\"pId\":"+pId+",\"type\":\""+eventType+"\",\"variables\":{";

    for (Map.Entry<String, String> entry : variables.entrySet()) {
      String key = entry.getKey();
      String value = entry.getValue();
      eventStr += "\"" + key + "\": " + value + ",";
    }
    eventStr = eventStr.substring(0, eventStr.length() - 1);
    eventStr += "},\"g\":"+g+",\"f\":"+f+"}";
    return eventStr;
  }

  //Generating
  public void generateEvent(String id,String parentId,HashMap<String,String> variables,double g, double f){
    nodeVariables.put(id, variables);
    parentNodes.put(id, parentId);
    nodeValues.put(id, new double[]{g, f});
    eventString += getEventString(EventType.GENERATING.toString(), id, parentId, variables, g, f) + ", ";
  }

  //Updating TODO If updating shape is needed during search then update this to be like generating
  public void updateEvent(String id,String pId,double g, double f){
    HashMap<String, String> variables = nodeVariables.get(id);
    nodeValues.put(id, new double[]{g, f});
    eventString += getEventString(EventType.UPDATING.toString(), id, pId, variables, g, f) + ", ";
  }

  public void expandNode(String id){
    HashMap<String,String> variables = nodeVariables.get(id);
    String parentId = parentNodes.get(id);
    double[] nValues = nodeValues.get(id);
    eventString += getEventString(EventType.EXPANDING.toString(), id, parentId, variables, nValues[0], nValues[1]) + ", ";
  }

  public void closeNode(String id){
    HashMap<String,String> variables = nodeVariables.get(id);
    String parentId = parentNodes.get(id);
    double[] nValues = nodeValues.get(id);
    eventString += getEventString(EventType.CLOSING.toString(), id, parentId, variables, nValues[0], nValues[1]) + ", ";
  }

  void addSmallEvent(String id, EventType type){
    if(eventString.length() > 15){
      eventString += ",\n";
    }
    eventString += "{\"id\":\""+id+"\",\"type\":\""+type.toString()+"\"}";
  }

  public void outputDebugFile(){
    // String objectString = svgObject.getOutputString();
    outputString += nodeString+",\n"+eventString+"]}";
    try{
      //print to document
      File file = new File("Test.json");

      FileWriter writer = new FileWriter(file);

      writer.write(outputString);
      writer.flush();
      writer.close();
    }
    catch (IOException e) {
      System.out.println("Write error");
    }
  }
}
