package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class NodePosition {
    private final double xCoord;
    private final double yCoord;
    private final double extension;
    private final double cannonAngle;

    public double getXCoord() {
        return xCoord;
    }
    public double getYCoord() {
        return yCoord;
    }
    public double getExtension() {
        return extension;
    }
    public double getCannonAngle(){
      return cannonAngle;

    }

    private NodePosition(double xCoord, double yCoord, double extension, double cannonAngle) {
      this.xCoord = xCoord;
      this.yCoord = yCoord;
      this.extension = extension;
      this.cannonAngle = cannonAngle;
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
      LOW_LEFT(0, -22, 0, "leftLow", 180, 37.5),
      LOW_CENTER(0, 0, 0, "midLow", 180, 100),
      LOW_RIGHT(0, 22, 0,"rightLow", 180, 37.5),
  
      MID_LEFT( 9.7, -22, 0, "leftMid", 139.5, 37.5),
      MID_CENTER(7.5, 0, 0, "midMid", 154.3, 100),
      MID_RIGHT(19.7, 22, 0,"rightMid", 139.5, 37.5),
  
  
      HIGH_LEFT(40, -22, 0, "leftHigh", 141.3, 37.5),
      HIGH_CENTER(38.0, 0, 0, "midHigh", 141.3, 100),
      HIGH_RIGHT(40, 22, 0, "rightHigh", 141.3, 37.5);
      
  
      public final double extension;
      public final double xOffset;
      public final double yOffset;
      public final String widgetName;
      public final double cannonAngle;
      public final double lidPosition;
      private NodeGrid(double extension, double yOffset, double xOffset, String widgetName, double cannonAngle, double lidPosition) {
        this.extension = extension;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.cannonAngle = cannonAngle;
        this.widgetName = widgetName;
        this.lidPosition = lidPosition;
      }

      public double getNodeCannonAngle(){
        return cannonAngle;
      }

      public double getNodeLidPosition(){
        return lidPosition;
      }
    }

  public static NodePosition getNodePosition(NodeGroup nodeGroup, NodeGrid nodeGrid) {
      double x = nodeGroup.xCoord + nodeGrid.xOffset + (DriverStation.getAlliance() == Alliance.Blue ? 99.86 : 0);
      return new NodePosition(x, nodeGroup.yCoord + nodeGrid.yOffset, nodeGrid.extension, nodeGrid.cannonAngle);
  }
}