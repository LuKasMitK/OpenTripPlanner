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
import org.opentripplanner.util.I18NString;
import org.opentripplanner.util.NonLocalizedString;

class StationTest {

  private static final String ID = "1";
  private static final I18NString NAME = new NonLocalizedString("name");
  private static final I18NString DESCRIPTION = new NonLocalizedString("description");
  private static final Station PARENT_STATION = TransitModelForTest.station("stationId").build();
  private static final String CODE = "code";

  public static final WgsCoordinate COORDINATE = new WgsCoordinate(0, 0);
  private static final StopTransferPriority PRIORITY = StopTransferPriority.ALLOWED;
  private static final TimeZone TIMEZONE = TimeZone.getTimeZone(TransitModelForTest.TIME_ZONE_ID);
  private static final I18NString URL = new NonLocalizedString("url");
  private static final Station subject = Station
    .of(TransitModelForTest.id(ID))
    .withName(NAME)
    .withDescription(DESCRIPTION)
    .withCode(CODE)
    .withCoordinate(COORDINATE)
    .withPriority(PRIORITY)
    .withTimezone(TIMEZONE)
    .withUrl(URL)
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
    assertEquals(DESCRIPTION, copy.getDescription());
    assertEquals(CODE, copy.getCode());
    assertTrue(COORDINATE.sameLocation(copy.getCoordinate()));
    assertEquals(PRIORITY, copy.getPriority());
    assertEquals(TIMEZONE, copy.getTimezone());
    assertEquals(URL, copy.getUrl());
  }

  @Test
  void sameAs() {
    assertTrue(subject.sameAs(subject.copy().build()));
    assertFalse(subject.sameAs(subject.copy().withId(TransitModelForTest.id("X")).build()));
    assertFalse(subject.sameAs(subject.copy().withName(new NonLocalizedString("X")).build()));
    assertFalse(subject.sameAs(subject.copy().withCode("X").build()));
    assertFalse(
      subject.sameAs(subject.copy().withDescription(new NonLocalizedString("X")).build())
    );
    assertFalse(
      subject.sameAs(subject.copy().withPriority(StopTransferPriority.DISCOURAGED).build())
    );
    assertFalse(subject.sameAs(subject.copy().withCoordinate(new WgsCoordinate(1, 1)).build()));
    assertFalse(subject.sameAs(subject.copy().withUrl(new NonLocalizedString("X")).build()));
    assertFalse(
      subject.sameAs(
        subject
          .copy()
          .withTimezone(TimeZone.getTimeZone(TransitModelForTest.OTHER_TIME_ZONE_ID))
          .build()
      )
    );
  }
}
