package frc.robot.subsystems.swerve;

public class Vector2D implements IVector2D {
    private double x = 0;
    private double y = 0;
    private double magnitude = 0;
    private double angleRadians = 0;

    public double getX() {
        return x;
    }
    
    public double getY() {
        return y;
    }

    public double getMagnitude() {
        return magnitude;
    }

    public double getAngleRadians() {
        return angleRadians;
    }

    public void setXY(double x, double y) {
        this.x = x;
        this.y = y;
        this.magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        this.angleRadians = Math.atan2(y, x);
    }

    public void setPolar(double magnitude, double angleRadians) {
        this.x = magnitude * Math.cos(angleRadians);
        this.y = magnitude * Math.sin(angleRadians);
        this.magnitude = magnitude;
        this.angleRadians = angleRadians;
    }
    public void copy(IVector2D vector) {
        this.x = vector.getX();
        this.y = vector.getY();
        this.magnitude = vector.getMagnitude();
        this.angleRadians = vector.getAngleRadians();
    }
    public void add(IVector2D vector1, IVector2D vector2) {
        var x = vector1.getX() + vector2.getX();
        var y = vector1.getY() + vector2.getY();
        this.setXY(x, y);

    }
    public void subtract(IVector2D vector1, IVector2D vector2) {
        var x = vector1.getX() - vector2.getX();
        var y = vector1.getY() - vector2.getY();
        this.setXY(x, y);
    }

    public void setMagnitude(double magnitude) {
        this.setPolar(magnitude, this.angleRadians);
    }
    public void rotate(double thetaRadians) {
        this.setPolar(magnitude, this.angleRadians + thetaRadians);
    }

    public void toUnitVector() {
        this.setMagnitude(1.0);
    }

    public static Vector2D FromXY(double x, double y) {
        var result = new Vector2D();
        result.setXY(x,y);
        return result;
    }
    public static Vector2D FromPolar(double magnitude, double angleRadians) {
        var result = new Vector2D();
        result.setPolar(magnitude, angleRadians);
        return result;
    }

    public void squareMagnitude() {
        var newMagnitude = Math.abs((this.magnitude) * this.magnitude);
        this.setPolar(newMagnitude, this.angleRadians);
    }
}
