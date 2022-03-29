package org.opentripplanner.graph_builder.module;

import java.io.Serializable;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import javax.annotation.Nonnull;

import org.onebusaway.gtfs.model.AgencyAndId;
import org.opentripplanner.graph_builder.services.GraphBuilderModule;
import org.opentripplanner.routing.algorithm.AStar;
import org.opentripplanner.routing.algorithm.strategies.InterleavedBidirectionalHeuristic;
import org.opentripplanner.routing.algorithm.strategies.MultiTargetTerminationStrategy;
import org.opentripplanner.routing.algorithm.strategies.RemainingWeightHeuristic;
import org.opentripplanner.routing.algorithm.strategies.SearchTerminationStrategy;
import org.opentripplanner.routing.algorithm.strategies.TrivialRemainingWeightHeuristic;
import org.opentripplanner.routing.core.RoutingContext;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.core.TraverseMode;
import org.opentripplanner.routing.core.TraverseModeSet;
import org.opentripplanner.routing.edgetype.TripPattern;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.TransferPattern;
import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.impl.PathComparator;
import org.opentripplanner.routing.spt.DominanceFunction;
import org.opentripplanner.routing.spt.GraphPath;
import org.opentripplanner.routing.spt.ShortestPathTree;
import org.opentripplanner.routing.vertextype.PatternArriveVertex;
import org.opentripplanner.routing.vertextype.TransitStationStop;
import org.opentripplanner.routing.vertextype.TransitStop;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.collect.HashMultimap;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Multimap;
import com.google.common.collect.Sets;

/**
 *
 * @author Sebastian Peter
 *
 */
public class TransferPatternsModule implements GraphBuilderModule, Serializable {
    private static final long serialVersionUID = 1L;

    private static final Logger LOG = LoggerFactory.getLogger(TransferPatternsModule.class);

    private static final double DEFAULT_MAX_WALK = 2000;
    private static final double CLAMP_MAX_WALK = 15000;

    private Multimap<TransitStationStop, TripPatternWithPosition> stopsToRoute = HashMultimap.create();

    private Map<TransitStationStop, TransferPattern> transferPatterns = new HashMap<>();

    @Override
    public void buildGraph(Graph graph, HashMap<Class<?>, Object> extra) {
        long startTime = System.currentTimeMillis();

        LOG.info("Building direct connection table...");
        {
            Set<TripPattern> tripPatterns = Sets.newHashSet();
            for (PatternArriveVertex pav : Iterables.filter(graph.getVertices(), PatternArriveVertex.class)) {
                tripPatterns.add(pav.getTripPattern());
            }

            for (TripPattern ttp : tripPatterns) {
                handleTripPattern(ttp);
            }
        }
        {
            LOG.info("Building transfer patterns...");
            Set<TransitStationStop> allStops = Sets.newHashSet();
            for (TransitStationStop s : Iterables.filter(graph.getVertices(), TransitStationStop.class)) {
                allStops.add(s);
            }

            computeTransferPatterns(graph, allStops);
        }

        long endTime = System.currentTimeMillis();
        LOG.info(String.format("Building transfer patterns took %.1f minutes.", (endTime - startTime) / 1000 / 60.0));
    }

    private void handleTripPattern(TripPattern tp) {
        for (int i = 0; i < tp.stopVertices.length; i++) {
            TransitStop tstop = tp.stopVertices[i];

            TripPatternWithPosition tpp = new TripPatternWithPosition(tp, i);
            stopsToRoute.put(tstop, tpp);
        }
    }

    private void computeTransferPatterns(Graph graph, Set<TransitStationStop> stops) {
        AStar aStar = new AStar();
        for (TransitStationStop root : stops) {
            RoutingRequest req = new RoutingRequest();
            req.maxTransfers = 2;
            req.modes = new TraverseModeSet(TraverseMode.BUS, TraverseMode.BUSISH, TraverseMode.RAIL,
                    TraverseMode.SUBWAY, TraverseMode.TRAINISH, TraverseMode.TRAM, TraverseMode.TRANSIT);
            // traversal modes
            req.modes.setWalk(true);
            // Now we always use what used to be called longDistance mode.
            // Non-longDistance mode is no longer supported.
            req.longDistance = true;
            req.numItineraries = 1;
            // set for one-to-all search
            req.batch = true;
            // no real-time data in transfer patterns
            req.ignoreRealtimeUpdates = true;

            req.rctx = new RoutingContext(req, graph, root, null);

            // since we search one-to-all, we do not need a real heuristic
            req.rctx.remainingWeightHeuristic = new TrivialRemainingWeightHeuristic();
            // TODO

            SearchTerminationStrategy terminationStrategy = new MultiTargetTerminationStrategy(new HashSet<>(stops));

            ShortestPathTree spt = aStar.getShortestPathTree(req, -1, terminationStrategy);
            if (spt != null) {
                transferPatterns.put(root, buildTP(spt, root));
            }
        }
    }

    private TransferPattern buildTP(@Nonnull ShortestPathTree spt, TransitStationStop root) {
        TransferPattern tp = new TransferPattern(root);
        // TODO caluculate tp


        return tp;
    }

    @Override
    public void checkInputs() {
        // TODO Auto-generated method stub

    }

    /**
     * serialization
     */
    public void save() {
        // TODO
    }

    public class TripPatternWithPosition {
        public TripPattern tripPattern;
        public int position;

        public TripPatternWithPosition(TripPattern tripPattern, int position) {
            this.tripPattern = tripPattern;
            this.position = position;
        }
    }

}
