/* This file is based on code copied from project OneBusAway, see the LICENSE file for further information. */
package org.opentripplanner.transit.model.network;

import static java.util.Objects.requireNonNull;
import static java.util.Objects.requireNonNullElse;

import java.util.List;
import java.util.Objects;
import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import org.opentripplanner.transit.model.framework.FeedScopedId;
import org.opentripplanner.transit.model.framework.LogInfo;
import org.opentripplanner.transit.model.framework.TransitEntity2;
import org.opentripplanner.transit.model.organization.Agency;
import org.opentripplanner.transit.model.organization.Branding;
import org.opentripplanner.transit.model.organization.Operator;

public final class Route extends TransitEntity2<Route, RouteBuilder> implements LogInfo {

  private final Agency agency;
  private final Operator operator;
  private final Branding branding;
  private final List<GroupOfRoutes> groupsOfRoutes;
  private final String shortName;
  private final String longName;
  private final TransitMode mode;
  // TODO: consolidate gtfsType and netexSubmode
  private final Integer gtfsType;
  private final Integer gtfsSortOrder;
  private final SubMode netexSubmode;
  private final String flexibleLineType;
  private final String description;
  private final String url;
  private final String color;
  private final String textColor;
  private final BikeAccess bikesAllowed;

  public Route(RouteBuilder builder) {
    super(builder.getId());
    // Required fields
    this.agency = requireNonNull(builder.getAgency());
    this.mode = requireNonNull(builder.getMode());
    this.bikesAllowed = requireNonNullElse(builder.getBikesAllowed(), BikeAccess.UNKNOWN);
    // One of short- or long- name is required
    this.shortName = builder.getShortName();
    this.longName = builder.getLongName();
    requireNonNull(getName());

    // Optional fields
    this.operator = builder.getOperator();
    this.branding = builder.getBranding();
    this.groupsOfRoutes = listOfNullSafe(builder.getGroupsOfRoutes());
    this.gtfsType = builder.getGtfsType();
    this.gtfsSortOrder = builder.getGtfsSortOrder();
    this.netexSubmode = SubMode.getOrBuildAndCacheForever(builder.getNetexSubmode());
    this.flexibleLineType = builder.getFlexibleLineType();
    this.description = builder.getDescription();
    this.url = builder.getUrl();
    this.color = builder.getColor();
    this.textColor = builder.getTextColor();
  }

  public static RouteBuilder of(@Nonnull FeedScopedId id) {
    return new RouteBuilder(id);
  }

  @Override
  public boolean sameAs(@Nonnull Route other) {
    return (
      getId().equals(other.getId()) &&
      Objects.equals(this.agency, other.agency) &&
      Objects.equals(this.operator, other.operator) &&
      Objects.equals(this.groupsOfRoutes, other.groupsOfRoutes) &&
      Objects.equals(this.shortName, other.shortName) &&
      Objects.equals(this.longName, other.longName) &&
      Objects.equals(this.branding, other.branding) &&
      Objects.equals(this.mode, other.mode) &&
      Objects.equals(this.gtfsType, other.gtfsType) &&
      Objects.equals(this.gtfsSortOrder, other.gtfsSortOrder) &&
      Objects.equals(this.flexibleLineType, other.flexibleLineType) &&
      Objects.equals(this.netexSubmode, other.netexSubmode) &&
      Objects.equals(this.description, other.description) &&
      Objects.equals(this.url, other.url) &&
      Objects.equals(this.color, other.color) &&
      Objects.equals(this.textColor, other.textColor) &&
      Objects.equals(this.bikesAllowed, other.bikesAllowed)
    );
  }

  @Override
  public RouteBuilder copy() {
    return new RouteBuilder(this);
  }

  /**
   * The 'agency' property represent a GTFS Agency and NeTEx the Authority. Note that Agency does
   * NOT map 1-1 to Authority, it is rather a mix between Authority and Operator.
   */
  @Nonnull
  public Agency getAgency() {
    return agency;
  }

  /**
   * NeTEx Operator, not in use when importing GTFS files.
   */
  @Nullable
  public Operator getOperator() {
    return operator;
  }

  @Nullable
  public Branding getBranding() {
    return branding;
  }

  @Nonnull
  public List<GroupOfRoutes> getGroupsOfRoutes() {
    return groupsOfRoutes;
  }

  @Nullable
  public String getShortName() {
    return shortName;
  }

  @Nullable
  public String getLongName() {
    return longName;
  }

  @Nonnull
  public TransitMode getMode() {
    return mode;
  }

  @Nullable
  public String getDescription() {
    return description;
  }

  @Nullable
  public Integer getGtfsType() {
    return gtfsType;
  }

  @Nullable
  public Integer getGtfsSortOrder() {
    return gtfsSortOrder;
  }

  @Nonnull
  public SubMode getNetexSubmode() {
    return netexSubmode;
  }

  @Nullable
  public String getUrl() {
    return url;
  }

  @Nullable
  public String getColor() {
    return color;
  }

  @Nullable
  public String getTextColor() {
    return textColor;
  }

  @Nonnull
  public BikeAccess getBikesAllowed() {
    return bikesAllowed;
  }

  /**
   * Pass-through information from NeTEx FlexibleLineType. This information is not used by OTP.
   */
  @Nullable
  public String getFlexibleLineType() {
    return flexibleLineType;
  }

  /** @return the route's short name, or the long name if the short name is null. */
  @Nonnull
  public String getName() {
    return shortName == null ? longName : shortName;
  }

  @Override
  public String logName() {
    return getName();
  }
}
