package org.opentripplanner.transit.model.site;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.TimeZone;
import org.junit.jupiter.api.Test;
import org.opentripplanner.transit.model._data.TransitModelForTest;
import org.opentripplanner.transit.model.basic.WgsCoordinate;
import org.opentripplanner.transit.model.basic.WheelchairAccessibility;
import org.opentripplanner.transit.model.network.SubMode;
import org.opentripplanner.transit.model.network.TransitMode;
import org.opentripplanner.util.I18NString;
import org.opentripplanner.util.NonLocalizedString;

class StopTest {

  private static final String ID = "1";
  private static final I18NString NAME = new NonLocalizedString("name");
  private static final I18NString DESCRIPTION = new NonLocalizedString("description");
  private static final Station PARENT_STATION = TransitModelForTest.station("stationId").build();
  private static final String CODE = "code";

  public static final WgsCoordinate COORDINATE = new WgsCoordinate(0, 0);
  private static final StopLevel LEVEL = new StopLevel("level", 0);
  private static final WheelchairAccessibility WHEELCHAIR_ACCESSIBILITY =
    WheelchairAccessibility.POSSIBLE;
  private static final String NETEX_SUBMODE_NAME = "submode";
  private static final SubMode NETEX_SUBMODE = SubMode.of(NETEX_SUBMODE_NAME);
  private static final TransitMode VEHICLE_TYPE = TransitMode.BUS;
  public static final TimeZone TIME_ZONE = TimeZone.getTimeZone(TransitModelForTest.TIME_ZONE_ID);
  private static final String PLATFORM_CODE = "platformCode";

  private static final Stop subject = Stop
    .of(TransitModelForTest.id(ID))
    .withName(NAME)
    .withDescription(DESCRIPTION)
    .withCode(CODE)
    .withCoordinate(COORDINATE)
    .withLevel(LEVEL)
    .withParentStation(PARENT_STATION)
    .withWheelchairAccessibility(WHEELCHAIR_ACCESSIBILITY)
    .withNetexVehicleSubmode(NETEX_SUBMODE_NAME)
    .withVehicleType(VEHICLE_TYPE)
    .withTimeZone(TIME_ZONE)
    .withPlatformCode(PLATFORM_CODE)
    .build();

  @Test
  void copy() {
    assertEquals(ID, subject.getId().getId());

    // Make a copy, and set the same name (nothing is changed)
    var copy = subject.copy().withName(NAME).build();

    assertSame(subject, copy);

    // Copy and change name
    copy = subject.copy().withName(new NonLocalizedString("v2")).build();

    // The two objects are not the same instance, but are equal(same id)
    assertNotSame(subject, copy);
    assertEquals(subject, copy);

    assertEquals(ID, copy.getId().getId());
    assertEquals("v2", copy.getName().toString());
    assertEquals(PARENT_STATION, copy.getParentStation());
    assertEquals(DESCRIPTION, copy.getDescription());
    assertEquals(CODE, copy.getCode());
    assertTrue(COORDINATE.sameLocation(copy.getCoordinate()));
    assertEquals(LEVEL.getName(), copy.getLevelName());
    assertEquals(LEVEL.getIndex(), copy.getLevelIndex());
    assertEquals(WHEELCHAIR_ACCESSIBILITY, copy.getWheelchairAccessibility());
    // TODO inconsistent naming
    assertEquals(NETEX_SUBMODE, copy.getNetexVehicleSubmode());
    assertEquals(VEHICLE_TYPE, copy.getGtfsVehicleType());
    assertEquals(TIME_ZONE, copy.getTimeZone());
    assertEquals(PLATFORM_CODE, copy.getPlatformCode());
  }

  @Test
  void sameAs() {
    assertTrue(subject.sameAs(subject.copy().build()));
    assertFalse(subject.sameAs(subject.copy().withId(TransitModelForTest.id("X")).build()));
    assertFalse(subject.sameAs(subject.copy().withName(new NonLocalizedString("X")).build()));
    assertFalse(
      subject.sameAs(subject.copy().withDescription(new NonLocalizedString("X")).build())
    );
    assertFalse(subject.sameAs(subject.copy().withCoordinate(new WgsCoordinate(1, 1)).build()));
    assertFalse(subject.sameAs(subject.copy().withUrl(new NonLocalizedString("X")).build()));
    assertFalse(subject.sameAs(subject.copy().withNetexVehicleSubmode("X").build()));
    assertFalse(subject.sameAs(subject.copy().withVehicleType(TransitMode.TRAM).build()));
    assertFalse(
      subject.sameAs(
        subject
          .copy()
          .withTimeZone(TimeZone.getTimeZone(TransitModelForTest.OTHER_TIME_ZONE_ID))
          .build()
      )
    );
    assertFalse(subject.sameAs(subject.copy().withPlatformCode("X").build()));
  }
}
