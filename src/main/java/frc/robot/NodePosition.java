package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class NodePosition {
    private final double xCoord;
    private final double yCoord;
    private final double extension;

    public double getXCoord() {
        return xCoord;
    }
    public double getYCoord() {
        return yCoord;
    }
    public double getExtension() {
        return extension;
    }
    private NodePosition(double xCoord, double yCoord, double extension) {
      this.xCoord = xCoord;
      this.yCoord = yCoord;
      this.extension = extension;
    }
    public enum NodeGroup{

      LEFT(0,42.19),
      CENTER(0,66+42.19),
      RIGHT(0,132+42.19),

      DOUBLE_SUBSTATION(3, 3),
      SINGLE_SUBSTATION(4, 4),
      GROUND_PICKUP(5, 5);

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
      HIGH_LEFT(60-27, -22, 0, "leftHigh"),
      MID_LEFT(45-27, -22, 0, "leftMid"),
      LOW_LEFT(0, -22, 0, "leftLow"),
      HIGH_CENTER(60-27, 0, 0, "midHigh"),
      MID_CENTER(45-27, 0, 0, "midMid"),
      LOW_CENTER(0, 0, 0, "midLow"),
      HIGH_RIGHT(60-27, 22, 0, "rightHigh"),
      MID_RIGHT(45-27, 22, 0,"rightMid"),
      LOW_RIGHT(0, 22, 0,"rightLow");

      public final double extension;
      public final double xOffset;
      public final double yOffset;
      public final String widgetName;
      private NodeGrid(double extension, double yOffset, double xOffset, String widgetName) {
        this.extension = extension;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.widgetName = widgetName;
      }
    }

  public static NodePosition getNodePosition(NodeGroup nodeGroup, NodeGrid nodeGrid) {
      double x = nodeGroup.xCoord + nodeGrid.xOffset + (DriverStation.getAlliance() == Alliance.Blue ? 99.86 : 0);
      return new NodePosition(x, nodeGroup.yCoord + nodeGrid.yOffset, nodeGrid.extension);
  }
}