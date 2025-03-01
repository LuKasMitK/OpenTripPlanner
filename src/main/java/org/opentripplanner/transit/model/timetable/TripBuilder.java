package org.opentripplanner.transit.model.timetable;

import org.opentripplanner.model.Direction;
import org.opentripplanner.model.TripAlteration;
import org.opentripplanner.transit.model.basic.WheelchairAccessibility;
import org.opentripplanner.transit.model.framework.AbstractEntityBuilder;
import org.opentripplanner.transit.model.framework.FeedScopedId;
import org.opentripplanner.transit.model.network.BikeAccess;
import org.opentripplanner.transit.model.network.Route;
import org.opentripplanner.transit.model.network.TransitMode;
import org.opentripplanner.transit.model.organization.Operator;

public class TripBuilder extends AbstractEntityBuilder<Trip, TripBuilder> {

  private Operator operator;
  private Route route;
  private FeedScopedId serviceId;
  private String shortName;
  private TransitMode mode;
  private String netexSubmode;
  private String headsign;
  private FeedScopedId shapeId;
  private Direction direction;
  private BikeAccess bikesAllowed;
  private WheelchairAccessibility wheelchairBoarding;
  private String gtfsBlockId;
  private String gtfsFareId;
  private String netexInternalPlanningCode;
  private TripAlteration netexAlteration;

  TripBuilder(FeedScopedId id) {
    super(id);
  }

  TripBuilder(Trip original) {
    super(original);
    this.route = original.getRoute();
    this.operator = original.getOperator();
    this.serviceId = original.getServiceId();
    this.mode = original.getMode();
    this.netexSubmode = original.getNetexSubMode().name();
    this.netexAlteration = original.getNetexAlteration();
    this.shortName = original.getShortName();
    this.headsign = original.getHeadsign();
    this.gtfsBlockId = original.getGtfsBlockId();
    this.shapeId = original.getShapeId();
    this.direction = original.getDirection();
    this.bikesAllowed = original.getBikesAllowed();
    this.wheelchairBoarding = original.getWheelchairBoarding();
    this.netexInternalPlanningCode = original.getNetexInternalPlanningCode();
    this.gtfsFareId = original.getGtfsFareId();
  }

  public Operator getOperator() {
    return operator;
  }

  public TripBuilder withOperator(Operator operator) {
    this.operator = operator;
    return this;
  }

  public Route getRoute() {
    return route;
  }

  public TripBuilder withRoute(Route route) {
    this.route = route;
    return this;
  }

  public FeedScopedId getServiceId() {
    return serviceId;
  }

  public TripBuilder withServiceId(FeedScopedId serviceId) {
    this.serviceId = serviceId;
    return this;
  }

  public String getShortName() {
    return shortName;
  }

  public TripBuilder withShortName(String shortName) {
    this.shortName = shortName;
    return this;
  }

  public TransitMode getMode() {
    return mode;
  }

  public TripBuilder withMode(TransitMode mode) {
    this.mode = mode;
    return this;
  }

  public String getNetexSubmode() {
    return netexSubmode;
  }

  public TripBuilder withNetexSubmode(String netexSubmode) {
    this.netexSubmode = netexSubmode;
    return this;
  }

  public String getNetexInternalPlanningCode() {
    return netexInternalPlanningCode;
  }

  public TripBuilder withNetexInternalPlanningCode(String netexInternalPlanningCode) {
    this.netexInternalPlanningCode = netexInternalPlanningCode;
    return this;
  }

  public String getHeadsign() {
    return headsign;
  }

  public TripBuilder withHeadsign(String headsign) {
    this.headsign = headsign;
    return this;
  }

  public String getGtfsBlockId() {
    return gtfsBlockId;
  }

  public TripBuilder withGtfsBlockId(String gtfsBlockId) {
    this.gtfsBlockId = gtfsBlockId;
    return this;
  }

  public FeedScopedId getShapeId() {
    return shapeId;
  }

  public TripBuilder withShapeId(FeedScopedId shapeId) {
    this.shapeId = shapeId;
    return this;
  }

  public Direction getDirection() {
    return direction;
  }

  public TripBuilder withDirection(Direction direction) {
    this.direction = direction;
    return this;
  }

  public BikeAccess getBikesAllowed() {
    return bikesAllowed;
  }

  public TripBuilder withBikesAllowed(BikeAccess bikesAllowed) {
    this.bikesAllowed = bikesAllowed;
    return this;
  }

  public WheelchairAccessibility getWheelchairBoarding() {
    return wheelchairBoarding;
  }

  public TripBuilder withWheelchairBoarding(WheelchairAccessibility wheelchairBoarding) {
    this.wheelchairBoarding = wheelchairBoarding;
    return this;
  }

  public TripAlteration getNetexAlteration() {
    return netexAlteration;
  }

  public TripBuilder withNetexAlteration(TripAlteration netexAlteration) {
    this.netexAlteration = netexAlteration;
    return this;
  }

  public String getGtfsFareId() {
    return gtfsFareId;
  }

  public TripBuilder withGtfsFareId(String gtfsFareId) {
    this.gtfsFareId = gtfsFareId;
    return this;
  }

  @Override
  protected Trip buildFromValues() {
    return new Trip(this);
  }
}
