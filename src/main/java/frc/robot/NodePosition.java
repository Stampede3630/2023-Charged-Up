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
    public enum NodeGroup{ // from the blue side
        LEFT(61.0+15,132+42.19), // 4.37 m for y????
        CENTER(61.0+15,66+42.19), // 2.65 meters for y??
        RIGHT(61.0+15,42.19); //.96 meters for y?? 37.79in??
      public final double xCoord;
      public final double yCoord;

      NodeGroup(double xCoord, double yCoord){
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
    public enum NodeGrid { //.5 meters offset??? from the blue side
      LOW_LEFT(0, 22, 0, 180, 37.5, 0.0, 150.0, -.3),
      LOW_CENTER(0, 0, 0, 180, 100, 0, 100, .3),
      LOW_RIGHT(0, -22, 0, 180, 37.5, 0.0, 150.0, -.3),
  
      MID_LEFT(17.5, 22, 0, 148.0, 150.75, 43.0, 37.0, -.5),
      MID_CENTER(3.5, 0, 0, 152.0, 28.0, 28.0, 28.0, .4), // cl3/21
      MID_RIGHT(17.5, -22, 0, 148.0, 150.75, 43.0, 37.0, -.5),

      HIGH_LEFT(36.0, 22, 0, 148.0, 164, 44.0, 37.0, -.5), //ej3/16
      HIGH_CENTER(19.0, 0, 0, 153.0, 28.0, 38.0, 28.0, 1), //cl3/21
      HIGH_RIGHT(36.0, -22, 0, 148.0, 164, 44.0, 37.0, -.5); //ej 3/16
      
  
      public final double extension;
      public final double xOffset;
      public final double yOffset;
      public final double lidUpCannonAngle;
      public final double lidUpLidPosition;
      public final double lidDownCannonAngle;
      public final double lidDownLidPosition;
      public final double intakeSpeed;
      private NodeGrid(double extension, double yOffset, double xOffset, double lidUpCannonAngle, double lidUpLidPosition, double lidDownCannonAngle, double lidDownLidPosition, double intakeSpeed) {
        this.extension = extension;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.lidUpCannonAngle = lidUpCannonAngle;
        this.lidUpLidPosition = lidUpLidPosition;
        this.lidDownCannonAngle = lidDownCannonAngle;
        this.lidDownLidPosition = lidDownLidPosition;
        this.intakeSpeed = intakeSpeed;
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