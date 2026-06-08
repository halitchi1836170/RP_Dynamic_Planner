using System.Collections.Generic;
using static MatrixVectorUtilities;
using UnityEngine;
using System;

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

    public List<PoseNode> getGraphNodesWithIDUpperBound(int upperBound)
    {
        List<PoseNode> result = new List<PoseNode>();

        foreach (PoseNode node in nodeList.Values)
        {
            if (node.PoseID() <= upperBound)
            {
                result.Add(node);
            }
        }
        return result;
    }


}
