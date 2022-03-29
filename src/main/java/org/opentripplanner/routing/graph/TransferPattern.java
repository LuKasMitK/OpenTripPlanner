package org.opentripplanner.routing.graph;

import java.util.LinkedList;
import java.util.List;

import org.opentripplanner.routing.vertextype.TransitStationStop;

/**
 *
 * @author Sebastian Peter
 *
 */
public class TransferPattern {
    private TPNode rootNode;

    private List<TPNode> targets = new LinkedList<>();

    public TransferPattern(TransitStationStop rootStop) {
        this.rootNode = new TPNode(rootStop);
    }

    /**
     * @return the rootNode
     */
    public TPNode getRootNode() {
        return rootNode;
    }

    public class TPNode {
        List<TPNode> predecessors = new LinkedList<>();
        private TransitStationStop stop;

        public TPNode(TransitStationStop stop) {
            this.stop = stop;
        }

        /**
         * @return the stop
         */
        public TransitStationStop getStop() {
            return stop;
        }

        public void addPredecessor(TPNode predecessor) {
            predecessors.add(predecessor);
        }

    }
}
