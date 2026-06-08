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

    private List<PoseNode> SearchLoopClosureCandidates_TimeAndConeBased(PoseNode currentNode, List<PoseNode> nodes)
    {
        List<PoseNode> returnList = new List<PoseNode>();
        Vector3 vCurrentNode = currentNode.getPoseTAsV3();

        // Direzione "avanti" del nodo corrente nel frame del grafo: forward = R * asse_forward_locale.
        // Assumendo che il forward locale del laser sia +Z (convenzione Unity), il forward e' la
        // TERZA COLONNA della rotazione di PoseT, cioe' (T[0,2], T[1,2], T[2,2]).
        // ATTENZIONE: se il cono seleziona i nodi DIETRO invece che davanti, il forward locale del
        //   tuo laser e' un altro asse -> usa +X = colonna 0 (T[*,0]) oppure +Y = colonna 1 (T[*,1]).
        float[,] T = currentNode.PoseT();
        Vector3 forward = new Vector3(T[0, 2], T[1, 2], T[2, 2]);
        if (forward.sqrMagnitude < 1e-12f) return returnList;
        forward.Normalize();

        float cosHalfAngle = Mathf.Cos(halfConeAngle * Mathf.Deg2Rad);

        foreach (PoseNode node in nodes)
        {
            Vector3 vNode = node.getPoseTAsV3();
            Vector3 vDif = vNode - vCurrentNode;
            float d = vDif.magnitude;

            // 1) entro il raggio massimo (ed evita il nodo stesso a distanza ~0)
            if (d > maxRadius || d < 1e-6f) continue;

            // 2) entro il cono frontale: angolo tra forward e direzione al candidato.
            //    cos(angolo) >= cos(semiangolo)  <=>  angolo <= semiangolo
            Vector3 dirToCandidate = vDif / d;
            float cosAngle = Vector3.Dot(forward, dirToCandidate);
            if (cosAngle >= cosHalfAngle)
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

    public List<PoseNode> getLoopClosureCandidates(LoopClosureFinder loopClosureMode, PoseNode currentNode, List<PoseNode> currentNodeKCandidates)
    {
        List<PoseNode> result = new List<PoseNode>();
        if (loopClosureMode == LoopClosureFinder.KNodeGapBased)
        {
            result = SearchLoopClosureCandidates_KNodeGapBased(currentNode, currentNodeKCandidates);
        }
        else
        {
            result = SearchLoopClosureCandidates_TimeAndConeBased(currentNode, currentNodeKCandidates);
        }
        return result;
    }

}
