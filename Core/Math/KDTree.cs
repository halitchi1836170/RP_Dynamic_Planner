using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor.Search;
using UnityEngine;

public class KDTree
{
    
    public class KDNode
    {
        private Vector3 node;
        private KDNode left; private KDNode right;
        private int originalIndex;

        public KDNode()
        {
            node = new Vector3();
            left = null;
            right = null;
            originalIndex = -1;
        }

        public KDNode(Vector3 node, KDNode left, KDNode right, int originalIndex)
        {
            this.node = node;
            this.left = left;
            this.right = right;
            this.originalIndex = originalIndex;
        }

        public KDNode(Vector3 node, int originalIndex)
        {
            this.node = node;
            this.originalIndex = originalIndex;
        }

        public KDNode getLeft()
        {
            return this.left;
        }

        public void Left(KDNode left)
        {
            this.left = left;
        }

        public void Right(KDNode right)
        {
            this.right = right;
        }   

        public KDNode getRight()
        {
            return this.right;
        }

        public int getOriginalIndex()
        {
            return this.originalIndex;
        }   

        public Vector3 getNode()
        {
            return this.node;
        }

    }

    private KDNode root;
    private int size;

    public float zeroTreshold = 0.001f;

    public KDTree()
    {
        this.root = new KDNode();
        this.size = 0;
    }

    public KDTree(KDNode root)
    {
        this.root = root;
        this.size=1;
    }

    public void BuildTree(List<Vector3> points, int depth)
    {
        if( points is null || points.Count == 0)   
        {
            Debug.Log("No points to process, returning...");
            return;
        }

        size = points.Count;

        List<(Vector3, int)> indexedPoints = new List<(Vector3, int)>(points.Count);
        for (int i = 0; i < size; i++)
        {   
            indexedPoints.Add((points[i], i));
        }

        this.root = BuildTreeRecursive(indexedPoints, depth);

    }

    private KDNode BuildTreeRecursive(List<(Vector3, int)> indexedPoints, int depth)
    {

        if (indexedPoints.Count == 0)
        {
            return null;
        }

        int axis = depth % 3;

        List<(Vector3, int)> sortedPoints = indexedPoints.OrderBy(p => p.Item1[axis]).ToList();

        int medianIndex = sortedPoints.Count / 2;

        Vector3 median = sortedPoints[medianIndex].Item1;
        int originalIndex = sortedPoints[medianIndex].Item2;

        KDNode node = new KDNode(median, originalIndex);

        node.Left(BuildTreeRecursive(sortedPoints.GetRange(0, medianIndex), depth + 1));
        node.Right(BuildTreeRecursive(sortedPoints.GetRange(medianIndex + 1, sortedPoints.Count - (medianIndex + 1)), depth + 1));

        return node;
    }

    public (Vector3, int, float) NearestNeighbor(Vector3 query)
    {
        if (root == null)
        {
            Debug.Log("Tree is empty, returning default values...");
            return (new Vector3(), -1, float.MaxValue);
        }

        KDNode bestNode = null;
        float bestDistSq = float.MaxValue;

        NearestNeighborRecursive(root, query, 0, ref bestNode, ref bestDistSq);

        return (bestNode.getNode(), bestNode.getOriginalIndex(), bestDistSq);
    }

    public (List<Vector3>, int[], float[]) KNearestNeighbor(Vector3 query,int k = 7)
    {
        if (root == null)
        {
            Debug.Log("Tree is empty, returning default values...");
            return (new List<Vector3>(), new int[] {},new float[] { });
        }

        List<(float, KDNode)> kBestNode = new List<(float, KDNode)>();
        float maxBestDistSq = float.MaxValue;

        KNearestNeighborRecursive(root, query, 0, ref kBestNode, ref maxBestDistSq, k);

        List<Vector3> returnVectorList = new List<Vector3>();
        int[] returnOriginalIndixes = new int[kBestNode.Count];
        float[] returnDistances = new float[kBestNode.Count];
        int counter = 0;
        foreach ((float, KDNode) nodeCouple in kBestNode) {
            returnVectorList.Add(nodeCouple.Item2.getNode());
            returnOriginalIndixes[counter] = nodeCouple.Item2.getOriginalIndex();
            returnDistances[counter] = nodeCouple.Item1;
            counter += 1;
        }

        return (returnVectorList, returnOriginalIndixes, returnDistances);
    }


    private void KNearestNeighborRecursive(KDNode node, Vector3 query, int depth, ref List<(float, KDNode)> kBestNode, ref float maxBestDistSq, int k)
    {
        if (node == null)
        {
            return;
        }

        float distSq = (node.getNode() - query).sqrMagnitude;
        if(kBestNode.Count == 0)
        {
            kBestNode.Add((distSq, node));
            maxBestDistSq = kBestNode[0].Item1;
        }
        else if (kBestNode.Count < k)
        {
            int counter = 0;
            while (counter < kBestNode.Count && kBestNode[counter].Item1 > distSq)
            {
                counter ++;
            }
            kBestNode.Insert(counter, (distSq, node));
            maxBestDistSq = kBestNode[0].Item1;
        }
        else if (distSq < maxBestDistSq && kBestNode.Count >= k) {

            kBestNode.RemoveAt(0);
            int counter = 0;
            while (counter < kBestNode.Count && kBestNode[counter].Item1 > distSq)
            {
                counter++;
            }
            kBestNode.Insert(counter, (distSq, node));
            maxBestDistSq = kBestNode[0].Item1;
        }

        int axis = depth % 3;
        float delta = GetAxis(query, axis) - GetAxis(node.getNode(), axis);

        KDNode first = delta < 0 ? node.getLeft() : node.getRight();
        KDNode second = delta < 0 ? node.getRight() : node.getLeft();

        KNearestNeighborRecursive(first, query, depth + 1, ref kBestNode, ref maxBestDistSq, k);

        if (delta * delta < maxBestDistSq)
        {
            KNearestNeighborRecursive(second, query, depth + 1, ref kBestNode, ref maxBestDistSq, k);
        }

    }

    private void NearestNeighborRecursive(KDNode node, Vector3 query, int depth, ref KDNode bestNode, ref float bestDistSq)
    {
        if (node == null)
        {
            return;
        }

        float distSq = (node.getNode() - query).sqrMagnitude;
        if(distSq < bestDistSq)
        {
            bestDistSq = distSq;
            bestNode = node;
        }

        int axis = depth % 3;
        float delta = GetAxis(query, axis) - GetAxis(node.getNode(),axis);

        KDNode first = delta < 0 ? node.getLeft() : node.getRight();
        KDNode second = delta < 0 ? node.getRight() : node.getLeft();

        NearestNeighborRecursive(first, query, depth + 1, ref bestNode, ref bestDistSq);

        if(delta * delta < bestDistSq)
        {
           NearestNeighborRecursive(second, query, depth+1, ref bestNode, ref bestDistSq);
        }

    }

    private float GetAxis(Vector3 v, int axis)
    {
        return axis == 0 ? v.x : axis == 1 ? v.y : v.z;
    }


    public List<(int queryIdx, int targetIdx, float distance)> FindCorrespondences(List<Vector3> queryCloud, float maxDistance = float.MaxValue)
    {
        List<(int, int, float)> correspondences = new List<(int, int, float)>(queryCloud.Count);

        for (int i = 0; i < queryCloud.Count; i++)
        {
            (Vector3 point, int targetIdx, float dist) = NearestNeighbor(queryCloud[i]);

            if(dist <= maxDistance)
            {
                correspondences.Add((i,targetIdx,dist));
            }
        }

        return correspondences;
    }

    public void DrawDebug(int maxDepth, float duration = 0.1f)
    {
        DrawDebugRecursive(root,0,maxDepth,duration);
    }

    public void DrawDebugRecursive(KDNode node, int depth, int maxDepth, float duration)
    {
        if (node is null) return;
        if(node.getLeft() != null)
        {
            Debug.DrawLine(node.getNode(), node.getLeft().getNode(), Color.Lerp(Color.white, Color.gray * 0.3f, (float)depth / maxDepth), duration);
            DrawDebugRecursive(node.getLeft(), depth + 1, maxDepth, duration);
        }
        if (node.getRight() != null)
        {
            Debug.DrawLine(node.getNode(), node.getRight().getNode(), Color.Lerp(Color.white, Color.gray * 0.3f, (float)depth / maxDepth), duration);
            DrawDebugRecursive(node.getRight(), depth + 1, maxDepth, duration);
        }
    }

    public List<(Vector3 from, Vector3 to, int depth)> GetEdges(int maxDepth = int.MaxValue)
    {
        var edges = new List<(Vector3, Vector3, int)>();
        GetEdgesRecursive(root, 0, edges, maxDepth);
        return edges;
    }

    private void GetEdgesRecursive(KDNode node, int depth, List<(Vector3, Vector3, int)> edges, int maxDepth)
    {
        if (node == null || depth >= maxDepth) return;
        if (node.getLeft() != null)
        {
            edges.Add((node.getNode(), node.getLeft().getNode(), depth));
            GetEdgesRecursive(node.getLeft(), depth + 1, edges, maxDepth);
        }
        if (node.getRight() != null)
        {
            edges.Add((node.getNode(), node.getRight().getNode(), depth));
            GetEdgesRecursive(node.getRight(), depth + 1, edges, maxDepth);
        }
    }

}
