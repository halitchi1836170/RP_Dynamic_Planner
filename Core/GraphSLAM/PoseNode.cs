using System.Collections.Generic;
using UnityEngine;

public class PoseNode
{
    
    private int poseID;
    private float[,] poseT;
    private List<Vector3> poseScannedPoints;
    private KDTree poseScannedPointsKDTree;


    public PoseNode()
    {
        poseID = -1;
        poseT = new float[4,4];
        poseScannedPoints = new List<Vector3>();
        poseScannedPointsKDTree = new KDTree();
    }

    public PoseNode(float[,] poseT, List<Vector3> scannedPoints, KDTree kdTree)
    {
        this.poseT = poseT;
        this.poseScannedPoints = scannedPoints;
        this.poseScannedPointsKDTree = kdTree;
    }

    public void setPoseID(int ID)
    {
        this.poseID = ID;
    }

    public int PoseID()
    {
        return poseID;
    }

    public float[,] PoseT() 
    { 
        return poseT;
    }

    public void setPoseT(float[,] newT)
    {
        if(poseT != null && newT.GetLength(0)==4 && newT.GetLength(1) == 4)
        {
            poseT = newT;
        }
    }

    public List<Vector3> PoseScannedPoints()
    {
        return poseScannedPoints;
    }

    public KDTree PoseScannedPointsKDTree()
    {
        return poseScannedPointsKDTree;
    }

}
