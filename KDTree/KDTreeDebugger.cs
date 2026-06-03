using UnityEngine;
using static KDTree;

public class KDTreeDebugger : MonoBehaviour
{
    public bool displayKDTree = false;

    private LiDAR3D mountedLidar;
    private KDTree kdTree;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        mountedLidar = this.GetComponent<LiDAR3D>();
        mountedLidar.OnScanComplete += DrawTree;
        kdTree = new KDTree();
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void DrawTree()
    {
        if (displayKDTree)
        {
            kdTree.BuildTree(mountedLidar.ScannedPoints, 0);
            kdTree.DrawDebug(15);
        }
    }
}
