package org.opentripplanner.util.lang;

import static java.lang.Boolean.TRUE;

import java.time.Duration;
import java.time.Instant;
import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Arrays;
import java.util.BitSet;
import java.util.Collection;
import java.util.Objects;
import java.util.function.Function;
import java.util.stream.Collectors;
import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import org.opentripplanner.util.time.DurationUtils;
import org.opentripplanner.util.time.TimeUtils;

/**
 * This toString builder which add elements to a compact string of the form:
 * <p>
 * {@code ClassName{field1:value, field2:value, ...}}
 * <p>
 * Fields equals to 'ignoreValue' is NOT added to the result string. This produces a short and easy
 * to read result. You should use {@code null} as 'ignoreValue' if the field is nullable.
 * <p>
 * The naming of the 'add' methods should give a hint to which type the value have, this make it
 * easier to choose the right method and less error prune as compared with relaying on pure
 * override, which often result in a wrong method call.
 * <p>
 * The builder should be independent of locale, the value should always be formatted the same way,
 * this allows us to use the toString in unit tests.
 */
public class ToStringBuilder {

  /** A random in value, not expected to exist in data */
  private static final int RANDOM_IGNORE_VALUE = -9_371_207;
  private static final String FIELD_SEPARATOR = ", ";
  private static final String FIELD_VALUE_SEP = ": ";
  private static final String NULL_VALUE = "null";

  private final StringBuilder sb = new StringBuilder();
  private final OtpNumberFormat numFormat = new OtpNumberFormat();

  boolean first = true;

  private ToStringBuilder(String name) {
    sb.append(name).append("{");
  }

  /**
   * Create a ToStringBuilder for a regular POJO type. This builder will include metadata(class and
   * field names) when building the to string.
   */
  public static ToStringBuilder of(Class<?> clazz) {
    return new ToStringBuilder(clazz.getSimpleName());
  }

  /**
   * Create a ToStringBuilder for a regular POJO type without including the type in the name. Some
   * classes are always embedded in other classes and the type is given, for these cases this
   * builder make the toString a bit easier to read.
   */
  public static ToStringBuilder of() {
    return new ToStringBuilder("");
  }

  /* General purpose formatters */

  public ToStringBuilder addNum(String name, Number num) {
    return addIfNotNull(name, num, numFormat::formatNumber);
  }

  public ToStringBuilder addNum(String name, Number value, Number ignoreValue) {
    return addIfNotIgnored(name, value, ignoreValue, numFormat::formatNumber);
  }

  public ToStringBuilder addNum(String name, Number num, String unit) {
    return addIfNotNull(name, num, n -> numFormat.formatNumber(n, unit));
  }

  public ToStringBuilder addBool(String name, Boolean value) {
    return addIfNotNull(name, value);
  }

  public ToStringBuilder addBoolIfTrue(String name, Boolean value) {
    if (TRUE.equals(value)) {
      addLabel(name);
    }
    return this;
  }

  public ToStringBuilder addStr(String name, String value) {
    return addIfNotNull(name, value, v -> "'" + v + "'");
  }

  public ToStringBuilder addEnum(String name, Enum<?> value) {
    return addEnum(name, value, null);
  }

  public ToStringBuilder addEnum(String name, Enum<?> value, Enum<?> ignoreValue) {
    return addIfNotIgnored(name, value, ignoreValue, Enum::name);
  }

  public ToStringBuilder addObj(String name, Object obj) {
    return addObj(name, obj, null);
  }

  public ToStringBuilder addObj(String name, Object value, @Nullable Object ignoreValue) {
    return addIfNotIgnored(name, value, ignoreValue, Object::toString);
  }

  /**
   * Use this if you would like a custom toString function to convert the value. If the given value
   * is null, then the value is not printed. The "Op" (Operation) suffix is necessary to separate
   * this from {@link #addObj(String, Object, Object)},  when the last argument is null.
   */
  public <T> ToStringBuilder addObjOp(
    String name,
    @Nullable T value,
    Function<T, Object> toObjectOp
  ) {
    return addIfNotIgnored(name, value, null, v -> nullSafeToString(toObjectOp.apply(v)));
  }

  public ToStringBuilder addInts(String name, int[] intArray) {
    return addIfNotNull(name, intArray, Arrays::toString);
  }

  public ToStringBuilder addDoubles(String name, double[] value, double ignoreValue) {
    if (value == null) {
      return addIt(name, "null");
    }
    if (Arrays.stream(value).allMatch(it -> Objects.equals(it, ignoreValue))) {
      return this;
    }
    return addIt(name, Arrays.toString(value));
  }

  /** Add collection if not null or not empty, all elements are added */
  public ToStringBuilder addCol(String name, Collection<?> c) {
    return addIfNotNull(name, c == null || c.isEmpty() ? null : c);
  }

  public ToStringBuilder addColSize(String name, Collection<?> c) {
    return addIfNotNull(name, c, x -> String.format("%d items", x.size()));
  }

  /** Add the collection, truncate the number of elements at given maxLimit. */
  public ToStringBuilder addCollection(String name, Collection<?> c, int maxLimit) {
    if (c == null) {
      return this;
    }
    if (c.size() > maxLimit + 1) {
      String value = c
        .stream()
        .limit(maxLimit)
        .map(Object::toString)
        .collect(Collectors.joining(", "));
      return addIt(name + "(" + maxLimit + "/" + c.size() + ")", "[" + value + ", ..]");
    }
    return addIfNotNull(name, c);
  }

  /** Add the collection, truncate the number of elements at given maxLimit. */
  public ToStringBuilder addIntArraySize(String name, int[] array, int notSet) {
    if (array == null) {
      return this;
    }
    return addIt(name, Arrays.stream(array).filter(t -> t != notSet).count() + "/" + array.length);
  }

  /** Add the BitSet: name : {cardinality}/{logical size}/{size} */
  public ToStringBuilder addBitSetSize(String name, BitSet bitSet) {
    if (bitSet == null) {
      return this;
    }
    return addIt(name, bitSet.cardinality() + "/" + bitSet.length());
  }

  /* Special purpose formatters */

  /** Add a Coordinate location, longitude or latitude */
  public ToStringBuilder addCoordinate(String name, Number num) {
    return addIfNotNull(name, num, numFormat::formatCoordinate);
  }

  /**
   * Add the TIME part in the local system timezone using 24 hours. Format:  HH:mm:ss. Note! The
   * DATE is not printed. {@code null} value is ignored.
   */
  public ToStringBuilder addTimeCal(String name, ZonedDateTime time) {
    return addIfNotNull(name, time, this::formatTime);
  }

  /**
   * Add time in seconds since midnight. Format:  hh:mm:ss. Ignore default values.
   */
  public ToStringBuilder addServiceTime(String name, int timeSecondsPastMidnight, int ignoreValue) {
    return addIfNotIgnored(name, timeSecondsPastMidnight, ignoreValue, TimeUtils::timeToStrCompact);
  }

  /**
   * Add time in seconds since midnight. Format:  hh:mm:ss.
   */
  public ToStringBuilder addServiceTime(String name, int timeSecondsPastMidnight) {
    return addIfNotIgnored(
      name,
      timeSecondsPastMidnight,
      RANDOM_IGNORE_VALUE,
      TimeUtils::timeToStrCompact
    );
  }

  /**
   * Add times in seconds since midnight. Format:  hh:mm. {@code null} value is ignored.
   */
  public ToStringBuilder addServiceTimeSchedule(String name, int[] value) {
    return addIfNotNull(
      name,
      value,
      a ->
        Arrays
          .stream(a)
          .mapToObj(TimeUtils::timeToStrCompact)
          .collect(Collectors.joining(" ", "[", "]"))
    );
  }

  public ToStringBuilder addTime(String name, Instant time) {
    return addIfNotNull(name, time, Instant::toString);
  }

  /**
   * Add a duration to the string in format like '3h4m35s'. Each component (hours, minutes, and or
   * seconds) is only added if they are not zero {@code 0}. This is the same format as the {@link
   * Duration#toString()}, but without the 'PT' prefix. {@code null} value is ignored.
   */
  public ToStringBuilder addDurationSec(String name, Integer durationSeconds) {
    return addDurationSec(name, durationSeconds, null);
  }

  /**
   * Add a duration to the string in format like '3h4m35s'. Each component (hours, minutes, and or
   * seconds) is only added if they are not zero {@code 0}. This is the same format as the {@link
   * Duration#toString()}, but without the 'PT' prefix. {@code null} value is ignored.
   */
  public ToStringBuilder addDurationSec(String name, Integer durationSeconds, Integer ignoreValue) {
    return addIfNotIgnored(name, durationSeconds, ignoreValue, DurationUtils::durationToStr);
  }

  public ToStringBuilder addDuration(String name, Duration duration) {
    return addIfNotIgnored(
      name,
      duration,
      null,
      d -> DurationUtils.durationToStr((int) d.toSeconds())
    );
  }

  @Override
  public String toString() {
    return sb.append("}").toString();
  }

  /** private methods */

  private <T> ToStringBuilder addIfNotNull(String name, T value) {
    return addIfNotIgnored(name, value, null, Object::toString);
  }

  private <T> ToStringBuilder addIfNotNull(String name, T value, Function<T, String> vToString) {
    return addIfNotIgnored(name, value, null, vToString);
  }

  private <T> ToStringBuilder addIfNotIgnored(
    String name,
    T value,
    T ignoreValue,
    Function<T, String> mapToString
  ) {
    // 'ignoreValue' should be the first argument here to avoid calling equals when
    // 'ignoreValue=null' and the type do not support equals(..).
    if (Objects.equals(ignoreValue, value)) {
      return this;
    }
    if (value == null) {
      return addIt(name, NULL_VALUE);
    }
    return addIt(name, mapToString.apply(value));
  }

  private ToStringBuilder addIt(String name, @Nonnull String value) {
    addLabel(name);
    addValue(value);
    return this;
  }

  private void addLabel(String name) {
    if (first) {
      first = false;
    } else {
      sb.append(FIELD_SEPARATOR);
    }
    sb.append(name);
  }

  private void addValue(@Nonnull String value) {
    sb.append(FIELD_VALUE_SEP);
    sb.append(value);
  }

  /**
   * Map the given object to a String. If the input object is {@code null} the string
   * {@code "null"} is returned if not the {@link Object#toString()} method is called.
   */
  public static String nullSafeToString(@Nullable Object object) {
    if (object == null) {
      return NULL_VALUE;
    }
    return object.toString();
  }

  private String formatTime(ZonedDateTime time) {
    return time.format(DateTimeFormatter.ISO_LOCAL_TIME);
  }
}
