using System.Collections.Generic;
using static MatrixVectorUtilities;
using UnityEngine;

public class PoseGraph
{
    private Dictionary<int,PoseNode> nodeList;
    private List<PoseEdge> edgeList;
    
    public PoseGraph() 
    {
        nodeList = new Dictionary<int, PoseNode>();
        edgeList = new List<PoseEdge>();
    }

    public void AddNode(PoseNode node)
    {
        nodeList[node.PoseID()]=node;
    }

    public void AddEdge(PoseEdge edge)
    {
        edgeList.Add(edge);
    }

    public IEnumerable<PoseNode> Nodes()
    {
        return nodeList.Values;
    }

    public List<PoseEdge> Edges()
    {
        return edgeList;
    }

    public PoseNode getNode(int i)
    {
        return nodeList[i];
    }

    public int getNNodes()
    {
        return nodeList.Count;
    }

    public List<PoseNode> SearchLoopClosureCandidates(PoseNode currentNode, int k,float loopClosureRadius)
    {
        List<PoseNode> returnList = new List<PoseNode>();
        foreach(PoseNode node in nodeList.Values)
        {
            if (currentNode.PoseID() - node.PoseID() >= k)
            {
                //Debug.Log($"Candidate node is k={k} steps far from the current node, calcualating distance...");
                float[] distanceVector = new float[3] { currentNode.PoseT()[0, 3] - node.PoseT()[0, 3], currentNode.PoseT()[1, 3] - node.PoseT()[1, 3], currentNode.PoseT()[2, 3] - node.PoseT()[2, 3] };
                if (getNormV3(distanceVector) <=loopClosureRadius)
                {
                    //Debug.Log($"Distance is {getNormV3(distanceVector)} and is smaller than loop closure radius: {loopClosureRadius}, adding node to candidates...");
                    returnList.Add(node);
                }
            }
        }
        return returnList;
    }
    

}
