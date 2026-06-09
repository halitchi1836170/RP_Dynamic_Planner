using System.Collections.Generic;
using UnityEngine;
using static MatrixVectorUtilities;

public class LoopClosureFinderService
{

    private float halfConeAngle;
    private int minIDGap;
    private float maxRadius;
    private float loopClosureThreshold;

    private float secondsFrequency;
    private float lastScan;

    public LoopClosureFinderService(float halfConeAngle, int minIDGap, float secondsFrequency, float maxRadius, float loopClosureThreshold)
    {
        this.halfConeAngle = halfConeAngle;
        this.minIDGap = minIDGap;
        this.secondsFrequency = secondsFrequency;
        this.lastScan = Time.time;
        this.maxRadius = maxRadius;
        this.loopClosureThreshold = loopClosureThreshold;
    }

    public bool itsTimeToFindLoopClosure()
    {
        if (Time.time - lastScan > secondsFrequency)
        {
            lastScan = Time.time;   // BUGFIX: senza il reset, dopo secondsFrequency ritorna sempre true
            return true;
        }
        return false;
    }

    // Cono di ricerca in Unity world e in 2D (piano orizzontale X-Z, Y = verticale ignorato).
    //  - robotPosWorld / robotForwardWorld: posa LIVE del robot (marrtinoLaserLinkTransform),
    //    così il forward è quello vero di Unity (niente indovinello sull'asse) ed è la direzione
    //    in cui stai realmente puntando.
    //  - nodeToWorld: converte la posa (drift-ata) di un nodo in Unity world (ICPToWorldPosition).
    //  Il test 2D rende la selezione robusta al drift verticale (i nodi rivisitati possono essere
    //  "scivolati" in alto/basso ma restano allineati sul piano).
    private List<PoseNode> SearchLoopClosureCandidates_TimeAndConeBased(Vector3 robotPosWorld, Vector3 robotForwardWorld, List<PoseNode> nodes, System.Func<PoseNode, Vector3> nodeToWorld)
    {
        List<PoseNode> returnList = new List<PoseNode>();

        Vector3 fwd = new Vector3(robotForwardWorld.x, 0f, robotForwardWorld.z);
        if (fwd.sqrMagnitude < 1e-12f) return returnList;
        fwd.Normalize();

        float cosHalfAngle = Mathf.Cos(halfConeAngle * Mathf.Deg2Rad);

        foreach (PoseNode node in nodes)
        {
            Vector3 w = nodeToWorld(node);
            Vector3 dir = new Vector3(w.x - robotPosWorld.x, 0f, w.z - robotPosWorld.z);   // proiezione 2D
            float d = dir.magnitude;

            // 1) entro il raggio massimo (ed evita il nodo stesso a distanza ~0)
            if (d > maxRadius || d < 1e-6f) continue;

            // 2) entro il cono frontale: angolo tra forward e direzione al candidato.
            //    cos(angolo) >= cos(semiangolo)  <=>  angolo <= semiangolo
            if (Vector3.Dot(fwd, dir / d) >= cosHalfAngle)
            {
                returnList.Add(node);
            }
        }
        return returnList;
    }


    private List<PoseNode> SearchLoopClosureCandidates_KNodeGapBased(PoseNode currentNode, List<PoseNode> kCandidates)
    {
        List<PoseNode> returnList = new List<PoseNode>();
        foreach (PoseNode node in kCandidates)
        {
            //Debug.Log($"Candidate node is k={k} steps far from the current node, calcualating distance...");
            float[] distanceVector = new float[3] { currentNode.PoseT()[0, 3] - node.PoseT()[0, 3], currentNode.PoseT()[1, 3] - node.PoseT()[1, 3], currentNode.PoseT()[2, 3] - node.PoseT()[2, 3] };
            if (getNormV3(distanceVector) <= maxRadius)   // BUGFIX: raggio spaziale, non la soglia di accettazione ICP
            {
                //Debug.Log($"Distance is {getNormV3(distanceVector)} and is smaller than loop closure radius, adding node to candidates...");
                returnList.Add(node);
            }
        }
        return returnList;
    }

    public List<PoseNode> getLoopClosureCandidates(LoopClosureFinder loopClosureMode, PoseNode currentNode, List<PoseNode> currentNodeKCandidates, Vector3 robotPosWorld, Vector3 robotForwardWorld, System.Func<PoseNode, Vector3> nodeToWorld)
    {
        List<PoseNode> result = new List<PoseNode>();
        if (loopClosureMode == LoopClosureFinder.KNodeGapBased)
        {
            result = SearchLoopClosureCandidates_KNodeGapBased(currentNode, currentNodeKCandidates);
        }
        else
        {
            result = SearchLoopClosureCandidates_TimeAndConeBased(robotPosWorld, robotForwardWorld, currentNodeKCandidates, nodeToWorld);
        }
        return result;
    }

}
