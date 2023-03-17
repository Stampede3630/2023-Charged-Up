package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class NodePosition {
    private final double xCoord;
    private final double yCoord;
    private final NodeGrid nodeGrid;
    private final NodeGroup nodeGroup;

    public double getXCoord() {
        return xCoord;
    }
    public double getYCoord() {
        return yCoord;
    }

    private NodePosition(double xCoord, double yCoord, NodeGrid nodeGrid, NodeGroup nodeGroup) {
      this.xCoord = xCoord;
      this.yCoord = yCoord;
      this.nodeGrid = nodeGrid;
      this.nodeGroup = nodeGroup;

    }
    public enum NodeGroup{

      LEFT(123.0,42.19),
      CENTER(123.0,66+42.19),
      RIGHT(123.0,132+42.19);

      public final double xCoord;
      public final double yCoord;

      private NodeGroup(double xCoord, double yCoord){
        this.xCoord = xCoord;
        this.yCoord = yCoord;
      }
    }

    /*in inches
    * BLUE
    * L 142.05
    * C 66 + 142.05
    * R 132 + 142.05
    * RED
    * L 42.19
    * C 66+ 42.19
    * R 132 + 42.19
    */
    public enum NodeGrid {
      LOW_LEFT(0, -22, 0, "leftLow", 180, 37.5, 0.0, 150.0),
      LOW_CENTER(0, 0, 0, "midLow", 0, 100, 0, 100),
      LOW_RIGHT(0, 22, 0,"rightLow", 180, 37.5, 0.0, 150.0),
  
      MID_LEFT(16.0, -22, 0, "leftMid", 40.0, 37.5, 26.0, 130.0),
      MID_CENTER(3.5, 0, 0, "midMid", 38.0, 100.0, 38.0, 100.0),
      MID_RIGHT(16.0, 22, 0,"rightMid", 40.0, 37.5, 26.0, 130.0),
  
  
      HIGH_LEFT(33.5, -22, 0, "leftHigh", 43.0, 37.5, 43.0, 40.0), //ej 3/16
      HIGH_CENTER(25.0, 0, 0, "midHigh", 38.7, 100.0, 38.7, 100.0), //ej3/15
      HIGH_RIGHT(33.5, 22, 0, "rightHigh", 43.0, 37.5, 43.0, 40.0); //ej3/16
      
  
      public final double extension;
      public final double xOffset;
      public final double yOffset;
      public final String widgetName;
      public final double lidUpCannonAngle;
      public final double lidUpLidPosition;
      public final double lidDownCannonAngle;
      public final double lidDownLidPosition;
      private NodeGrid(double extension, double yOffset, double xOffset, String widgetName, double lidDownCannonAngle, double lidDownLidPosition, double lidUpCannonAngle, double lidUpLidPosition) {
        this.extension = extension;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.lidUpCannonAngle = lidDownCannonAngle;
        this.widgetName = widgetName;
        this.lidUpLidPosition = lidDownLidPosition;
        this.lidDownCannonAngle = lidUpCannonAngle;
        this.lidDownLidPosition = lidUpLidPosition;
      }

      public double getNodeCannonAngleLidUp(){
        return lidUpCannonAngle;
      }

      public double getNodeLidPositionLidUp(){
        return lidUpLidPosition;
      }
      public double getNodeCannonAngleLidDown(){
        return lidDownCannonAngle;
      }

      public double getNodeLidPositionLidDown(){
        return lidDownLidPosition;
      }
      public double getExtension(){
        return extension;
      }
    }

  public static NodePosition getNodePosition(NodeGroup nodeGroup, NodeGrid nodeGrid) {
      double x = nodeGroup.xCoord + nodeGrid.xOffset + (DriverStation.getAlliance() == Alliance.Blue ? 99.86 : 0);
      return new NodePosition(x, nodeGroup.yCoord + nodeGrid.yOffset, nodeGrid, nodeGroup);
  }
}