using System.Collections.Generic;
using UnityEngine;

public class ICPUtils
{

    public static List<Vector3> ToLocalFrame(Transform marrtionLaserLinkTransform, List<Vector3> worldPoints)
    {
        List<Vector3> local = new List<Vector3>(worldPoints.Count);
        foreach (Vector3 p in worldPoints)
            local.Add(marrtionLaserLinkTransform.InverseTransformPoint(p));
        return local;
    }

}
