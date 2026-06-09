using System.Collections.Generic;
using UnityEngine;
using static ICPUtils;
using static MatrixVectorUtilities;
using static PoseMatrix4x4;
using static ICPSolver;

public class ICPService
{
    //INPUTED FIELDS
    private float deltaHuber;
    private ICPMode icpMode;
    private int maxIteration;
    private float maxDistance;
    private float convergenceThreshold;
    private float voxelSize;
    private int nPosesPath;
    private Transform marrtinoLaserLinkTransform;
    private LiDAR3D lidar3d;
    private GraphSlamService graphSlamService;

    //CALCULATED FIELDS
    private float[,] TWorldICP;
    private float[,] TRelativeICP;

    private KDTree kdTargetTree;
    private KDTree kdSourceTree;
    private List<Vector3> targetLocalTreeListOfPoints;       //downsampled
    private List<Vector3> sourceLocalTreeListOfPoints;       //downsampled

    private VoxelGrid targetVoxelGrid;
    private VoxelGrid sourceVoxelGrid;

    private Vector3 firstMarrtinoLocation;
    private Quaternion firstMarrtinoRotation;

    private List<Vector3> transformedWorldPointsList;
    private Queue<Vector3> icpPathPositions;
    private Vector3 icpEstimatedPos;

    private bool planarConstraintFlag;

    public ICPService(ICPMode icpMode, float deltaHuber, int maxIteration, float maxDistance, float convergenceThreshold, float voxelSize, int nPosesPath, Transform marrtinoLaserLinkTransform, LiDAR3D lidar, GraphSlamService graphSlamService, bool planarConstraintFlag)
    {
        this.icpMode = icpMode;
        this.deltaHuber = deltaHuber;
        this.maxIteration = maxIteration;
        this.maxDistance = maxDistance;
        this.convergenceThreshold = convergenceThreshold;
        this.voxelSize = voxelSize;
        this.nPosesPath = nPosesPath;
        this.marrtinoLaserLinkTransform = marrtinoLaserLinkTransform;
        this.lidar3d = lidar;
        this.graphSlamService = graphSlamService;
        this.planarConstraintFlag = planarConstraintFlag;

        TWorldICP = Identity4();
        TRelativeICP = Identity4();
        targetVoxelGrid = new VoxelGrid();
        sourceVoxelGrid = new VoxelGrid();

        icpPathPositions = new Queue<Vector3>();
        transformedWorldPointsList = new List<Vector3>();
    }

    public (Vector3, Queue<Vector3>, List<Vector3>) ScanCompletedRunOneICP()
    {

        if (kdTargetTree == null)
        {
            kdTargetTree = new KDTree();
            targetLocalTreeListOfPoints = ToLocalFrame(marrtinoLaserLinkTransform, targetVoxelGrid.Downsample(lidar3d.ScannedPoints, voxelSize));
            kdTargetTree.BuildTree(targetLocalTreeListOfPoints, 0);
            marrtinoLaserLinkTransform.GetPositionAndRotation(out firstMarrtinoLocation, out firstMarrtinoRotation);
            icpEstimatedPos = marrtinoLaserLinkTransform.position;
            PoseNode nodo = new PoseNode(Identity4(), targetLocalTreeListOfPoints, kdTargetTree);
            graphSlamService.updateLastNode(nodo);
            graphSlamService.insertNode(nodo);
            return (new Vector3(), new Queue<Vector3>(), new List<Vector3>());
        }

        kdSourceTree = new KDTree();
        sourceLocalTreeListOfPoints = ToLocalFrame(marrtinoLaserLinkTransform, sourceVoxelGrid.Downsample(lidar3d.ScannedPoints, voxelSize));
        kdSourceTree.BuildTree(sourceLocalTreeListOfPoints, 0);

        float[,] deltaT = Solve(TRelativeICP, kdTargetTree, targetLocalTreeListOfPoints, sourceLocalTreeListOfPoints, deltaHuber, maxIteration, maxDistance, convergenceThreshold, icpMode);
        
        
        if (planarConstraintFlag)
        {
            float[,] T_candidate = projectPoseToPlane(productSquareMatrix4(TWorldICP, deltaT));
            deltaT = productSquareMatrix4(InverseT(TWorldICP), T_candidate);   // relativo coerente con la posa planare
            TWorldICP = T_candidate;
            TRelativeICP = deltaT;
        }
        else
        {
            TRelativeICP = deltaT;
            TWorldICP = productSquareMatrix4(TWorldICP, TRelativeICP);
        }
        

        // ICP path (cyan): accumula la traslazione stimata
        Vector3 localDelta = new Vector3(TRelativeICP[0, 3], TRelativeICP[1, 3], TRelativeICP[2, 3]);
        Vector3 worldDelta = marrtinoLaserLinkTransform.TransformVector(localDelta);
        icpEstimatedPos += worldDelta;
        
        if (icpPathPositions.Count == nPosesPath)
            icpPathPositions.Dequeue();
        icpPathPositions.Enqueue(icpEstimatedPos);

        transformedWorldPointsList.Clear();
        Vector3 transformedP;
        foreach (Vector3 p in sourceLocalTreeListOfPoints)
        {
            transformedP = applyTransformation(TWorldICP, p);
            transformedWorldPointsList.Add(firstMarrtinoRotation * transformedP + firstMarrtinoLocation);
        }

        targetLocalTreeListOfPoints = sourceLocalTreeListOfPoints;
        kdTargetTree = kdSourceTree;

        return (icpEstimatedPos, icpPathPositions, transformedWorldPointsList);

    }

    public float[,] getTWorldICP()
    {
        return TWorldICP;
    }

    public float[,] getTRelativeICP()
    {
        return TRelativeICP;
    }

    public List<Vector3> getSourceLocalTreeListOfPoints()
    {
        return sourceLocalTreeListOfPoints;
    }

    public KDTree getSourceKDTree()
    {
        return kdSourceTree;
    }

    public float[,] SolveInverseICPProblem(float[,] initialGuess, PoseNode candidate, List<Vector3> sourceScannedPoints)
    {
        float[,] deltaTCandidate = ICPSolver.Solve(initialGuess, candidate.PoseScannedPointsKDTree(), candidate.PoseScannedPoints(), sourceScannedPoints, deltaHuber, maxIteration, maxDistance, convergenceThreshold, icpMode);
        if (planarConstraintFlag)
        {
            // La misura di loop closure deve essere planare come l'odometria, altrimenti vincoli
            // planari e non-planari si scontrano e l'ottimizzatore distorce il grafo (tilt/verticale).
            // E' una posa relativa tra due nodi planari: stessa proiezione (verticale = [1,3], up = Y).
            deltaTCandidate = projectPoseToPlane(deltaTCandidate);
        }
        return deltaTCandidate;
    }

    public void setTWorldICP(float[,] correctedT)
    {
        TWorldICP = correctedT;
    }

    public Vector3 ICPToWorldPosition(float px, float py, float pz)
    {
        return firstMarrtinoRotation * new Vector3(px, py, pz) + firstMarrtinoLocation;
    }

    public List<Vector3> ICPToWorldPositionEnumerableNodes(IEnumerable<PoseNode> nodes)
    {
        List<Vector3> result = new List<Vector3>();
        foreach (PoseNode n in nodes)
            result.Add(ICPToWorldPosition(n.PoseT()[0, 3], n.PoseT()[1, 3], n.PoseT()[2, 3]));
        return result;
    }

    public List<Vector3> ICPToWorldPositionListVectors(List<Vector3> vectors)
    {
        List<Vector3> result = new List<Vector3>();
        foreach (Vector3 v in vectors)
            result.Add(ICPToWorldPosition(v.x, v.y, v.z));
        return result;
    }

    public List<(Vector3 from, Vector3 to)> ICPToWorldPositionPairs(List<(Vector3 from, Vector3 to)> pairs)
    {
        List<(Vector3, Vector3)> result = new List<(Vector3, Vector3)>();
        foreach ((Vector3 from, Vector3 to) p in pairs)
            result.Add((ICPToWorldPosition(p.from.x, p.from.y, p.from.z), ICPToWorldPosition(p.to.x, p.to.y, p.to.z)));
        return result;
    }

}
