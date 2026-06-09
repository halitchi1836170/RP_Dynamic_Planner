using System;
using System.Collections.Generic;
using UnityEngine;
using static MatrixVectorUtilities;
using static PoseMatrix4x4;

public enum LoopClosureFinder { KNodeGapBased, TimeAndConeBased };

public class GraphSlamService
{
    private int nodeCounter;
    private int edgeCounter;
    private PoseGraph poseGraph;
    private PoseNode newNode;
    private PoseNode lastNode;
    private PoseEdge newEdge;

    private int maxIterations;
    private int maxCGIterations;
    private float convergenceThreshold;
    private float convergenceCGThreshold;

    private float minDeltaTranslation;
    private float minDeltaRotation;

    private List<Vector3> globalUpdatedMap;
    private List<(int from, int to)> closureEdgesWorldPositions;
    private HashSet<(int from, int to)> hashLoopClosureFounded;

    private float[,] AccumulateedTRelativeForNodeInsertion;

    private float huberDeltaGraphSlamOptimization;

    public GraphSlamService(int maxIterationsNumber, int maxCGIterations, float graphSlamoptimizerThreshold, float CGConvergenceTrheshold, float minDeltaT, float minDeltaR, float huberDeltaGraphSlamOptimization)
    {
        nodeCounter = 0;
        edgeCounter = 0;
        poseGraph = new PoseGraph();
        this.maxIterations = maxIterationsNumber;
        this.maxCGIterations = maxCGIterations;
        this.convergenceThreshold = graphSlamoptimizerThreshold;
        this.convergenceCGThreshold = CGConvergenceTrheshold;
        this.minDeltaRotation = minDeltaR;
        this.minDeltaTranslation = minDeltaT;
        globalUpdatedMap = new List<Vector3>();
        this.AccumulateedTRelativeForNodeInsertion = Identity4();
        this.closureEdgesWorldPositions = new List<(int from, int to)>();
        this.hashLoopClosureFounded = new HashSet<(int, int)>();
        this.huberDeltaGraphSlamOptimization = huberDeltaGraphSlamOptimization;
    }

    public void insertNode(PoseNode node)
    {
        nodeCounter++;
        node.setPoseID(nodeCounter);
        poseGraph.AddNode(node);
    }

    public void insertEdge(PoseEdge edge)
    {
        newEdge = edge;
        poseGraph.AddEdge(edge);
        edgeCounter++;
    }

    public int getNodeCounter()
    {
        return this.nodeCounter;
    }

    public void updateLastNode(PoseNode node)
    {
        lastNode = node;
    }

    public void updateLastNode()
    {
        lastNode = newNode;
    }

    public void updateNewNode(PoseNode node)
    {
        newNode = node;
    }

    public void updateGraphSLAMWithNewNode(float[,] TWorldICP, List<Vector3> sourceLocalTreeListOfPoints, KDTree kdSourceTree)
    {
        newNode = new PoseNode(TWorldICP, sourceLocalTreeListOfPoints, kdSourceTree);
        insertNode(newNode);
        // La misura dell'edge deve essere il moto relativo ACCUMULATO dall'ultimo nodo
        // (T_i^-1 * T_j), non il singolo step ICP TRelativeICP: tra due nodi ci sono N scansioni
        // e usare solo l'ultima contrarrebbe la catena di ~N volte in ottimizzazione.
        newEdge = new PoseEdge(lastNode.PoseID(), newNode.PoseID(), AccumulateedTRelativeForNodeInsertion, Identity6());
        insertEdge(newEdge);
        AccumulateedTRelativeForNodeInsertion = Identity4();
    }

    public bool updateGraphSLAMWithClosureEdge(PoseNode candidate, float[,] deltaTCandidate)
    {
        // deltaTCandidate = T_candidate^-1 * T_newNode (l'ICP allinea i punti di newNode su candidate).
        // L'ottimizzatore si aspetta z_ij = T_from^-1 * T_to, quindi l'edge deve andare
        // da candidate (from) a newNode (to) per rispettare la convenzione del residuo.
        // Ritorna true solo se l'edge e' stato effettivamente aggiunto (coppia non gia' chiusa):
        // serve all'orchestrator per non ri-ottimizzare a vuoto sui trigger ripetuti.
        if(hashLoopClosureFounded.Add((candidate.PoseID(), newNode.PoseID())))
        {
            PoseEdge closureEdge = new PoseEdge(candidate.PoseID(), newNode.PoseID(), deltaTCandidate, Identity6());
            closureEdge.setLoopClosure(true);
            insertEdge(closureEdge);
            closureEdgesWorldPositions.Add((candidate.PoseID(), newNode.PoseID()));
            return true;
        }
        return false;
    }

    internal float[,] GetInitialGuessForCandidate(PoseNode candidate)
    {
        return productSquareMatrix4(InverseT(candidate.PoseT()), newNode.PoseT());
    }

    internal List<Vector3> getNewNodeScannedPoints()
    {
        return newNode.PoseScannedPoints();
    }

    public void OptimizeGraph()
    {
        GraphSlamOptimizer graphOptimizer = new GraphSlamOptimizer(maxIterations, maxCGIterations, convergenceThreshold, convergenceCGThreshold, poseGraph, huberDeltaGraphSlamOptimization);
        graphOptimizer.Optimize();
    }

    public void updateGlobalScannedPointsAfterOptimization()
    {
        globalUpdatedMap.Clear();
        foreach(PoseNode node in poseGraph.Nodes())
        {
            List<Vector3> nodeScannedPoints = node.PoseScannedPoints();
            foreach (Vector3 point in nodeScannedPoints)
            {
                Vector3 new_point = applyTransformation(node.PoseT(), point);
                globalUpdatedMap.Add(new_point);
            }
        }
    }

    public List<Vector3> getUpdatedGlobalMapPointCloud()
    {
        return globalUpdatedMap;
    }

    public IEnumerable<PoseNode> getGraphNodes()
    {
        return poseGraph.Nodes();
    }

    public List<PoseNode> getGraphNodesWithIDUpperBound(int k)
    {
        return poseGraph.getGraphNodesWithIDUpperBound(k);
    }

    public PoseNode getNewNode()
    {
        return newNode;
    }

    // Moto relativo accumulato dall'ultimo nodo inserito (T_lastNode^-1 * T_robotCorrente).
    // Serve al re-anchoring post-ottimizzazione quando il trigger scatta mentre il robot si e'
    // gia' mosso oltre l'ultimo nodo (caso tipico del trigger temporale TimeAndCone).
    public float[,] getAccumulatedRelativeSinceLastNode()
    {
        return AccumulateedTRelativeForNodeInsertion;
    }

    public bool checkIfMoovedSinceLastNode(float[,] relativeICP)
    {
        AccumulateedTRelativeForNodeInsertion = productSquareMatrix4(AccumulateedTRelativeForNodeInsertion, relativeICP);

        float[] se3Vector = LogMap(AccumulateedTRelativeForNodeInsertion);
        float[] measuredTraslation = new float[3] { se3Vector[0], se3Vector[1], se3Vector[2] };
        float[] measuredRotation = new float[3] { se3Vector[3], se3Vector[4], se3Vector[5] };

        bool hasMooved = getNormV3(measuredTraslation) > minDeltaTranslation || getNormV3(measuredRotation) > minDeltaRotation;
        return hasMooved;
    }

    public List<(Vector3 from, Vector3 to)> getClosureEdgeVector3Couples()
    {
        List<(int from, int to)> idCouplesList = getClosureEdgesIDCouples();
        List<(Vector3 from, Vector3 to)> result = new List<(Vector3, Vector3)>();
        foreach((int from, int to) couple in idCouplesList)
        {
            result.Add((poseGraph.getNode(couple.from).getPoseTAsV3(), poseGraph.getNode(couple.to).getPoseTAsV3()));
        }
        return result;
    }

    public List<(int from, int to)> getClosureEdgesIDCouples()
    {
        return closureEdgesWorldPositions;
    }

}
