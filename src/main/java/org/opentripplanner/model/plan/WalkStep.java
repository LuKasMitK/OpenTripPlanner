package org.opentripplanner.model.plan;

import com.google.common.collect.Lists;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.opentripplanner.common.model.P2;
import org.opentripplanner.model.StreetNote;
import org.opentripplanner.model.VehicleRentalStationInfo;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.transit.model.basic.WgsCoordinate;
import org.opentripplanner.util.I18NString;
import org.opentripplanner.util.lang.DoubleUtils;

/**
 * Represents one instruction in walking directions. Three examples from New York City:
 * <p>
 * Turn onto Broadway from W 57th St (coming from 7th Ave): <br> distance = 100 (say) <br>
 * walkDirection = RIGHT <br> streetName = Broadway <br> everything else null/false <br>
 * </p>
 * <p>
 * Now, turn from Broadway onto Central Park S via Columbus Circle <br> distance = 200 (say) <br>
 * walkDirection = CIRCLE_COUNTERCLOCKWISE <br> streetName = Central Park S <br> exit = 1 (first
 * exit) <br> immediately everything else false <br>
 * </p>
 * <p>
 * Instead, go through the circle to continue on Broadway <br> distance = 100 (say) <br>
 * walkDirection = CIRCLE_COUNTERCLOCKWISE <br> streetName = Broadway <br> exit = 3 <br> stayOn =
 * true <br> everything else false <br>
 * </p>
 */
public class WalkStep {

  private double distance = 0.0;
  private RelativeDirection relativeDirection;
  private I18NString streetName;
  private AbsoluteDirection absoluteDirection;

  private final Set<StreetNote> streetNotes = new HashSet<>();

  private final Boolean area;
  private final Boolean bogusName;
  private final WgsCoordinate startLocation;
  private final double angle;
  private final boolean walkingBike;

  private String exit;
  private List<P2<Double>> elevation;
  private Boolean stayOn = false;

  private List<Edge> edges = Lists.newArrayList();

  private VehicleRentalStationInfo vehicleRentalOnStation;

  private VehicleRentalStationInfo vehicleRentalOffStation;

  public WalkStep(
    I18NString streetName,
    WgsCoordinate startLocation,
    boolean bogusName,
    double angle,
    boolean walkingBike,
    boolean area
  ) {
    this.streetName = streetName;
    this.startLocation = startLocation;
    this.bogusName = bogusName;
    this.angle = DoubleUtils.roundTo2Decimals(angle);
    this.walkingBike = walkingBike;
    this.area = area;
  }

  public void setDirections(double lastAngle, double thisAngle, boolean roundabout) {
    relativeDirection = RelativeDirection.calculate(lastAngle, thisAngle, roundabout);
    setAbsoluteDirection(thisAngle);
  }

  public void setAbsoluteDirection(double thisAngle) {
    int octant = (int) (8 + Math.round(thisAngle * 8 / (Math.PI * 2))) % 8;
    absoluteDirection = AbsoluteDirection.values()[octant];
  }

  /**
   * The elevation profile as a comma-separated list of x,y values. x is the distance from the start
   * of the step, y is the elevation at this distance.
   */
  public List<P2<Double>> getElevation() {
    return elevation;
  }

  public void addElevation(List<P2<Double>> elevation) {
    if (elevation == null) {
      return;
    }
    if (this.elevation == null) {
      this.elevation = new ArrayList<>();
    }
    this.elevation.addAll(StreetLeg.normalizeElevation(elevation));
  }

  public void addStreetNotes(Collection<StreetNote> streetNotes) {
    if (streetNotes == null) {
      return;
    }
    this.streetNotes.addAll(streetNotes);
  }

  public String streetNameNoParens() {
    var str = streetName.toString();
    if (str == null) {
      return null; //Avoid null reference exceptions with pathways which don't have names
    }
    int idx = str.indexOf('(');
    if (idx > 0) {
      return str.substring(0, idx - 1);
    }
    return str;
  }

  public Set<StreetNote> getStreetNotes() {
    return streetNotes;
  }

  /**
   * The distance in meters that this step takes.
   */
  public double getDistance() {
    return distance;
  }

  public void addDistance(double distance) {
    this.distance = DoubleUtils.roundTo2Decimals(this.distance + distance);
  }

  /**
   * The relative direction of this step.
   */
  public RelativeDirection getRelativeDirection() {
    return relativeDirection;
  }

  public void setRelativeDirection(RelativeDirection relativeDirection) {
    this.relativeDirection = relativeDirection;
  }

  /**
   * The name of the street.
   */
  public I18NString getStreetName() {
    return streetName;
  }

  public void setStreetName(I18NString streetName) {
    this.streetName = streetName;
  }

  /**
   * The absolute direction of this step.
   */
  public AbsoluteDirection getAbsoluteDirection() {
    return absoluteDirection;
  }

  public void setAbsoluteDirection(AbsoluteDirection absoluteDirection) {
    this.absoluteDirection = absoluteDirection;
  }

  /**
   * When exiting a highway or traffic circle, the exit name/number.
   */
  public String getExit() {
    return exit;
  }

  public void setExit(String exit) {
    this.exit = exit;
  }

  /**
   * Indicates whether or not a street changes direction at an intersection.
   */
  public Boolean getStayOn() {
    return stayOn;
  }

  public void setStayOn(Boolean stayOn) {
    this.stayOn = stayOn;
  }

  /**
   * This step is on an open area, such as a plaza or train platform, and thus the directions should
   * say something like "cross"
   */
  public Boolean getArea() {
    return area;
  }

  /**
   * The name of this street was generated by the system, so we should only display it once, and
   * generally just display right/left directions
   */
  public Boolean getBogusName() {
    return bogusName;
  }

  /**
   * The coordinate of start of the step
   */
  public WgsCoordinate getStartLocation() {
    return startLocation;
  }

  public double getAngle() {
    return angle;
  }

  /**
   * Is this step walking with a bike?
   */
  public boolean isWalkingBike() {
    return walkingBike;
  }

  /**
   * The street edges that make up this walkStep. Used only in generating the streetEdges array in
   * StreetSegment; not serialized.
   */
  public List<Edge> getEdges() {
    return edges;
  }

  public void setEdges(List<Edge> edges) {
    this.edges = edges;
  }

  /**
   * The vehicle rental on/off station info. Used only in generating the streetEdges array in
   * StreetSegment; not serialized.
   */
  public VehicleRentalStationInfo getVehicleRentalOnStation() {
    return vehicleRentalOnStation;
  }

  public void setVehicleRentalOnStation(VehicleRentalStationInfo vehicleRentalOnStation) {
    this.vehicleRentalOnStation = vehicleRentalOnStation;
  }

  public VehicleRentalStationInfo getVehicleRentalOffStation() {
    return vehicleRentalOffStation;
  }

  public void setVehicleRentalOffStation(VehicleRentalStationInfo vehicleRentalOffStation) {
    this.vehicleRentalOffStation = vehicleRentalOffStation;
  }

  @Override
  public String toString() {
    String direction = absoluteDirection.toString();
    if (relativeDirection != null) {
      direction = relativeDirection.toString();
    }
    return "WalkStep(" + direction + " on " + streetName + " for " + distance + ")";
  }
}
