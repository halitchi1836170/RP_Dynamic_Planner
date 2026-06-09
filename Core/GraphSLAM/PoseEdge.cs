using UnityEngine;

public class PoseEdge
{

    private int fromNodeID;
    private int toNodeID;
    private float[,] relativeT;
    private float[,] informationM;
    private bool isLoopClosure;

    public PoseEdge()
    {
        fromNodeID = 0;
        toNodeID = 0;
        relativeT = new float[4, 4];
        informationM = new float[6, 6];
        isLoopClosure = false;
    }

    public PoseEdge(int fromNodeID, int toNodeID, float[,] relativeT, float[,] informationM)
    {
        this.fromNodeID = fromNodeID;
        this.toNodeID = toNodeID;
        this.relativeT = relativeT;
        this.informationM = informationM;
        this.isLoopClosure = false;
    }

    public int FromNodeID() { return fromNodeID; }

    public int ToNodeID() { return toNodeID; }

    public float[,] RelativeT() { return relativeT; }

    public float[,] InformationM() { return informationM; }

    public bool IsLoopClosure() {  return isLoopClosure; }

    public void setLoopClosure(bool vBool)
    {
        this.isLoopClosure = vBool;
    }

}
