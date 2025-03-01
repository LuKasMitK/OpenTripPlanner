package org.opentripplanner.ext.transmodelapi.model.plan;

import static org.opentripplanner.ext.transmodelapi.model.EnumTypes.ALTERNATIVE_LEGS_FILTER;
import static org.opentripplanner.ext.transmodelapi.model.EnumTypes.MODE;

import graphql.Scalars;
import graphql.scalars.ExtendedScalars;
import graphql.schema.DataFetchingEnvironment;
import graphql.schema.GraphQLArgument;
import graphql.schema.GraphQLFieldDefinition;
import graphql.schema.GraphQLList;
import graphql.schema.GraphQLNonNull;
import graphql.schema.GraphQLObjectType;
import graphql.schema.GraphQLOutputType;
import graphql.schema.GraphQLTypeReference;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.stream.Collectors;
import org.opentripplanner.ext.transmodelapi.model.EnumTypes;
import org.opentripplanner.ext.transmodelapi.model.TransmodelTransportSubmode;
import org.opentripplanner.ext.transmodelapi.model.TripTimeShortHelper;
import org.opentripplanner.ext.transmodelapi.support.GqlUtil;
import org.opentripplanner.model.calendar.ServiceDate;
import org.opentripplanner.model.plan.Leg;
import org.opentripplanner.model.plan.StopArrival;
import org.opentripplanner.model.plan.legreference.LegReferenceSerializer;
import org.opentripplanner.routing.RoutingService;
import org.opentripplanner.routing.alternativelegs.AlternativeLegs;
import org.opentripplanner.transit.service.TransitService;
import org.opentripplanner.util.PolylineEncoder;

public class LegType {

  private static final String NAME = "Leg";
  public static final GraphQLTypeReference REF = new GraphQLTypeReference(NAME);

  public static GraphQLObjectType create(
    GraphQLOutputType bookingArrangementType,
    GraphQLOutputType interchangeType,
    GraphQLOutputType linkGeometryType,
    GraphQLOutputType authorityType,
    GraphQLOutputType operatorType,
    GraphQLOutputType quayType,
    GraphQLOutputType estimatedCallType,
    GraphQLOutputType lineType,
    GraphQLOutputType serviceJourneyType,
    GraphQLOutputType datedServiceJourneyType,
    GraphQLOutputType ptSituationElementType,
    GraphQLObjectType placeType,
    GraphQLObjectType pathGuidanceType,
    GqlUtil gqlUtil
  ) {
    return GraphQLObjectType
      .newObject()
      .name("Leg")
      .description(
        "Part of a trip pattern. Either a ride on a public transport vehicle or access or path link to/from/between places"
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("id")
          .description("An identifier for the leg, which can be used to re-fetch the information.")
          .type(Scalars.GraphQLID)
          .dataFetcher(env -> LegReferenceSerializer.encode(leg(env).getLegReference()))
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("aimedStartTime")
          .description("The aimed date and time this leg starts.")
          .type(new GraphQLNonNull(gqlUtil.dateTimeScalar))
          .dataFetcher(env -> // startTime is already adjusted for realtime - need to subtract delay to get aimed time
            leg(env)
              .getStartTime()
              .minusSeconds(leg(env).getDepartureDelay())
              .toInstant()
              .toEpochMilli()
          )
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("expectedStartTime")
          .description("The expected, realtime adjusted date and time this leg starts.")
          .type(new GraphQLNonNull(gqlUtil.dateTimeScalar))
          .dataFetcher(env -> leg(env).getStartTime().toInstant().toEpochMilli())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("aimedEndTime")
          .description("The aimed date and time this leg ends.")
          .type(new GraphQLNonNull(gqlUtil.dateTimeScalar))
          .dataFetcher(env -> // endTime is already adjusted for realtime - need to subtract delay to get aimed time
            leg(env)
              .getEndTime()
              .minusSeconds(leg(env).getArrivalDelay())
              .toInstant()
              .toEpochMilli()
          )
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("expectedEndTime")
          .description("The expected, realtime adjusted date and time this leg ends.")
          .type(new GraphQLNonNull(gqlUtil.dateTimeScalar))
          .dataFetcher(env -> leg(env).getEndTime().toInstant().toEpochMilli())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("mode")
          .description(
            "The mode of transport or access (e.g., foot) used when traversing this leg."
          )
          .type(new GraphQLNonNull(MODE))
          .dataFetcher(env -> leg(env).getMode())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("transportSubmode")
          .description(
            "The transport sub mode (e.g., localBus or expressBus) used when traversing this leg. Null if leg is not a ride"
          )
          .type(EnumTypes.TRANSPORT_SUBMODE)
          .dataFetcher(environment ->
            ((Leg) environment.getSource()).getTrip() != null
              ? TransmodelTransportSubmode.fromValue(
                ((Leg) environment.getSource()).getTrip().getNetexSubMode()
              )
              : null
          )
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("duration")
          .description("The leg's duration in seconds")
          .type(new GraphQLNonNull(ExtendedScalars.GraphQLLong))
          .dataFetcher(env -> leg(env).getDuration())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("directDuration")
          .type(new GraphQLNonNull(ExtendedScalars.GraphQLLong))
          .description("NOT IMPLEMENTED")
          .dataFetcher(env -> leg(env).getDuration())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("pointsOnLink")
          .description("The leg's geometry.")
          .type(linkGeometryType)
          .dataFetcher(env -> PolylineEncoder.encodeGeometry(leg(env).getLegGeometry()))
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("authority")
          .description(
            "For ride legs, the service authority used for this legs. For non-ride legs, null."
          )
          .type(authorityType)
          .dataFetcher(env -> leg(env).getAgency())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("operator")
          .description("For ride legs, the operator used for this legs. For non-ride legs, null.")
          .type(operatorType)
          .dataFetcher(env -> leg(env).getOperator())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("realtime")
          .description("Whether there is real-time data about this leg")
          .type(new GraphQLNonNull(Scalars.GraphQLBoolean))
          .dataFetcher(env -> leg(env).getRealTime())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("distance")
          .description("The distance traveled while traversing the leg in meters.")
          .type(new GraphQLNonNull(Scalars.GraphQLFloat))
          .dataFetcher(env -> leg(env).getDistanceMeters())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("generalizedCost")
          .description("Generalized cost or weight of the leg. Used for debugging.")
          .type(Scalars.GraphQLInt)
          .dataFetcher(env -> leg(env).getGeneralizedCost())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("ride")
          .description("Whether this leg is a ride leg or not.")
          .type(new GraphQLNonNull(Scalars.GraphQLBoolean))
          .dataFetcher(env -> leg(env).isTransitLeg())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("walkingBike")
          .description("Whether this leg is walking with a bike.")
          .type(Scalars.GraphQLBoolean)
          .dataFetcher(env -> leg(env).getWalkingBike())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("rentedBike")
          .description("Whether this leg is with a rented bike.")
          .type(Scalars.GraphQLBoolean)
          .dataFetcher(env -> leg(env).getRentedVehicle())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("fromPlace")
          .description("The Place where the leg originates.")
          .type(new GraphQLNonNull(placeType))
          .dataFetcher(env -> leg(env).getFrom())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("toPlace")
          .description("The Place where the leg ends.")
          .type(new GraphQLNonNull(placeType))
          .dataFetcher(env -> leg(env).getTo())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("fromEstimatedCall")
          .withDirective(gqlUtil.timingData)
          .description("EstimatedCall for the quay where the leg originates.")
          .type(estimatedCallType)
          .dataFetcher(env -> TripTimeShortHelper.getTripTimeShortForFromPlace(env.getSource()))
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("toEstimatedCall")
          .withDirective(gqlUtil.timingData)
          .description("EstimatedCall for the quay where the leg ends.")
          .type(estimatedCallType)
          .dataFetcher(env -> TripTimeShortHelper.getTripTimeShortForToPlace(env.getSource()))
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("line")
          .description("For ride legs, the line. For non-ride legs, null.")
          .type(lineType)
          .dataFetcher(env -> leg(env).getRoute())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("serviceJourney")
          .description("For ride legs, the service journey. For non-ride legs, null.")
          .type(serviceJourneyType)
          .dataFetcher(env -> leg(env).getTrip())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("datedServiceJourney")
          .description("The dated service journey used for this leg.")
          .type(datedServiceJourneyType)
          .dataFetcher(env -> {
            var trip = leg(env).getTrip();
            if (trip == null) {
              return null;
            }
            var tripId = leg(env).getTrip().getId();
            var serviceDate = leg(env).getServiceDate();

            return GqlUtil
              .getTransitService(env)
              .getTripOnServiceDateForTripAndDay(tripId, serviceDate);
          })
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("serviceDate")
          .description(
            "For transit legs, the service date of the trip. For non-transit legs, null."
          )
          .type(gqlUtil.dateScalar)
          .dataFetcher(environment ->
            Optional
              .of((Leg) environment.getSource())
              .map(Leg::getServiceDate)
              .map(ServiceDate::toLocalDate)
              .orElse(null)
          )
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("intermediateQuays")
          .description(
            "For ride legs, intermediate quays between the Place where the leg originates and the Place where the leg ends. For non-ride legs, empty list."
          )
          .type(new GraphQLNonNull(new GraphQLList(new GraphQLNonNull(quayType))))
          .dataFetcher(env -> {
            List<StopArrival> stops = ((Leg) env.getSource()).getIntermediateStops();
            if (stops == null || stops.isEmpty()) {
              return List.of();
            } else {
              return (
                stops
                  .stream()
                  .map(stop -> stop.place.stop)
                  .filter(Objects::nonNull)
                  .collect(Collectors.toList())
              );
            }
          })
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("intermediateEstimatedCalls")
          .withDirective(gqlUtil.timingData)
          .description(
            "For ride legs, estimated calls for quays between the Place where the leg originates and the Place where the leg ends. For non-ride legs, empty list."
          )
          .type(new GraphQLNonNull(new GraphQLList(new GraphQLNonNull(estimatedCallType))))
          .dataFetcher(env ->
            TripTimeShortHelper.getIntermediateTripTimeShortsForLeg((env.getSource()))
          )
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("serviceJourneyEstimatedCalls")
          .withDirective(gqlUtil.timingData)
          .description(
            "For ride legs, all estimated calls for the service journey. For non-ride legs, empty list."
          )
          .type(new GraphQLNonNull(new GraphQLList(new GraphQLNonNull(estimatedCallType))))
          .dataFetcher(env -> TripTimeShortHelper.getAllTripTimeShortsForLegsTrip(env.getSource()))
          .build()
      )
      //                .field(GraphQLFieldDefinition.newFieldDefinition()
      //                        .name("via")
      //                        .description("Do we continue from a specified via place")
      //                        .type(Scalars.GraphQLBoolean)
      //                        .dataFetcher(env -> leg(env).intermediatePlace)
      //                        .build())
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("situations")
          .description("All relevant situations for this leg")
          .type(new GraphQLNonNull(new GraphQLList(new GraphQLNonNull(ptSituationElementType))))
          .dataFetcher(env -> leg(env).getTransitAlerts())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("steps")
          .description("Do we continue from a specified via place")
          .type(new GraphQLNonNull(new GraphQLList(pathGuidanceType)))
          .dataFetcher(env -> leg(env).getWalkSteps())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("interchangeFrom")
          .type(interchangeType)
          .dataFetcher(env -> leg(env).getTransferFromPrevLeg())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("interchangeTo")
          .type(interchangeType)
          .dataFetcher(env -> leg(env).getTransferToNextLeg())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("bookingArrangements")
          .type(bookingArrangementType)
          .dataFetcher(env -> leg(env).getPickupBookingInfo())
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("bikeRentalNetworks")
          .type(new GraphQLNonNull(new GraphQLList(Scalars.GraphQLString)))
          .dataFetcher(env ->
            leg(env).getVehicleRentalNetwork() == null
              ? List.of()
              : List.of(leg(env).getVehicleRentalNetwork())
          )
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("previousLegs")
          .description(
            "Fetch the previous legs, which can be used to replace this leg. The replacement legs do arrive/depart from/to the same stop places. It might be necessary to change other legs in an itinerary in order to be able to ride the returned legs."
          )
          .type(new GraphQLList(new GraphQLNonNull(REF)))
          .argument(
            GraphQLArgument
              .newArgument()
              .name("previous")
              .description("Number of earlier legs to return.")
              .defaultValueProgrammatic(1)
              .type(Scalars.GraphQLInt)
          )
          .argument(
            GraphQLArgument
              .newArgument()
              .name("filter")
              .description("Whether the leg should be similar to this leg in some way.")
              .defaultValueProgrammatic("noFilter")
              .type(ALTERNATIVE_LEGS_FILTER)
              .build()
          )
          .dataFetcher(env -> {
            Leg leg = leg(env);
            if (!leg.isScheduledTransitLeg()) {
              return null;
            }
            int previous = env.getArgument("previous");
            RoutingService routingService = GqlUtil.getRoutingService(env);
            TransitService transitService = GqlUtil.getTransitService(env);
            return AlternativeLegs.getAlternativeLegs(
              leg,
              previous,
              routingService,
              transitService,
              true,
              env.getArgument("filter")
            );
          })
          .build()
      )
      .field(
        GraphQLFieldDefinition
          .newFieldDefinition()
          .name("nextLegs")
          .description(
            "Fetch the next legs, which can be used to replace this leg. The replacement legs do arrive/depart from/to the same stop places. It might be necessary to change other legs in an itinerary in order to be able to ride the returned legs."
          )
          .type(new GraphQLList(new GraphQLNonNull(REF)))
          .argument(
            GraphQLArgument
              .newArgument()
              .name("next")
              .description("Number of later legs to return.")
              .defaultValueProgrammatic(1)
              .type(Scalars.GraphQLInt)
          )
          .argument(
            GraphQLArgument
              .newArgument()
              .name("filter")
              .description("Whether the leg should be similar to this leg in some way.")
              .defaultValueProgrammatic("noFilter")
              .type(ALTERNATIVE_LEGS_FILTER)
              .build()
          )
          .dataFetcher(env -> {
            Leg leg = leg(env);
            if (!leg.isScheduledTransitLeg()) {
              return null;
            }
            int next = env.getArgument("next");
            RoutingService routingService = GqlUtil.getRoutingService(env);
            TransitService transitService = GqlUtil.getTransitService(env);
            return AlternativeLegs.getAlternativeLegs(
              leg,
              next,
              routingService,
              transitService,
              false,
              env.getArgument("filter")
            );
          })
          .build()
      )
      .build();
  }

  private static Leg leg(DataFetchingEnvironment environment) {
    return environment.getSource();
  }
}
