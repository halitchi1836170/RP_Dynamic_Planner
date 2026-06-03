using System.Collections.Generic;
using UnityEngine;
using static MatrixVectorUtilities;
using static PoseMatrix4x4;

public enum ICPMode { P2P, P2C }

public class ICPSolver
{
    public static float[,] Solve(float[,] InitialGuessT,KDTree kdTargetTree, List<Vector3> target, List<Vector3> source, float deltaHuber, int maxIteration, float maxDistance, float convergenceThreshold, ICPMode mode)
    {
        float[,] returnT = InitialGuessT;
        for (int iter = 0; iter < maxIteration; iter++)
        {
            float[,] H = Zeros66();
            float[] b = Zeros6();
            float cumulativeError = 0.0f;
            int pointCounter = 0;
            foreach(Vector3 point in source)
            {
                Vector3 point_new = applyTransformation(returnT, point);

                if (mode == ICPMode.P2P)
                {
                    (bool validGuess, float[,] updatedH, float[]updatedB, float[] errore) returnP2P = computeICPPoint2Point(point_new, kdTargetTree, maxDistance, H, b);
                    if (returnP2P.validGuess)
                    {
                        H = returnP2P.updatedH;
                        b = returnP2P.updatedB;
                        cumulativeError += Mathf.Pow(getNormV3(returnP2P.errore), 2);
                    }

                }else if (mode == ICPMode.P2C)
                {
                    (bool validGuess, float[,] updatedH, float[] updatedB, float[] errore) returnP2C = computeICPPoint2Cloud(point_new, kdTargetTree, maxDistance, H, b, deltaHuber);
                    if (returnP2C.validGuess)
                    {
                        H = returnP2C.updatedH;
                        b = returnP2C.updatedB;
                        cumulativeError += Mathf.Pow(getNormV3(returnP2C.errore), 2);
                    }
                }

                pointCounter += 1;
            }

            float lambda = 1e-5f;
            for (int k = 0; k < 6; k++) H[k, k] += lambda;
            float[] deltaP = Solve6x6(H, b);
            returnT = productSquareMatrix4(returnT, ExpMap(deltaP));
            //Debug.Log($"IteratioNn: {iter + 1}, cumulativeError mean error: {cumulativeError/(pointCounter)}, getNorm(deltaP): {getNorm(deltaP)}");
            if (getNorm(deltaP) < convergenceThreshold){
                break;
            }
            
        }

        return returnT;
    }

    private static (bool, float[,], float[], float[]) computeICPPoint2Cloud(Vector3 point_new, KDTree kdTargetTree, float maxDistance, float[,] H, float[] b, float delta)
    {
        (List<Vector3> kNearestQ, int[] indexes, float[] distances) returnKNearestSearch = kdTargetTree.KNearestNeighbor(point_new);
        if (Mathf.Sqrt(returnKNearestSearch.distances[0]) > maxDistance)
        {
            return (false, new float[,] { }, new float[] { }, new float[] { });
        }

        Vector3 centroide = avgListVector3(returnKNearestSearch.kNearestQ);

        float[,] covarianceMatrix = getCovarianceMatrix3(returnKNearestSearch.kNearestQ);
        float[] nVector = getMinimumEigenvector(covarianceMatrix);

        float[] errore = new float[] { point_new.x - centroide.x, point_new.y - centroide.y, point_new.z - centroide.z };
        float erroreScalare = DottProductV3(errore, nVector);
        
        //float weight = (float)(1.0 / (1.0 + avgArrayFloats(returnKNearestSearch.distances)));
        float weight = 0.0f;
        if ( Mathf.Abs(erroreScalare) <= delta)
        {
            weight = 1.0f;
        }
        else
        {
            weight = delta / Mathf.Abs(erroreScalare);
        }
        
        (float[,] updatedH, float[] updatedB) accumulated = AccumulateHbP2C(point_new, erroreScalare, nVector, weight, H, b);
        return (true, accumulated.updatedH, accumulated.updatedB, errore);
    }

    private static (float[,], float[]) AccumulateHbP2C(Vector3 point_new, float erroreScalare, float[] nVector, float weight, float[,] H, float[] b)
    {
        float[] jacobian = GetJacobianP2C(nVector, point_new);
        float[,] Hreturn = sumSquareMatrix6(H, productSquareMatrix6Scalar(productColumnRowVector6(jacobian, jacobian), weight));
        float[] bReturn = sumVector6(b, productVector6Scalar(jacobian, erroreScalare * weight));
        return (Hreturn, bReturn);
    }

    private static float[] GetJacobianP2C(float[] nVector, Vector3 point_new)
    {
        float[] jacobian = new float[6]; 
        
        jacobian[0] = nVector[0];
        jacobian[1] = nVector[1];
        jacobian[2] = nVector[2];

        float[] prodVettoriale = CrossProductV3(new float[3] { point_new.x, point_new.y, point_new.z}, nVector);

        jacobian[3] = prodVettoriale[0];
        jacobian[4] = prodVettoriale[1];
        jacobian[5] = prodVettoriale[2];

        return jacobian;
    }

    private static (bool,float[,], float[], float[]) computeICPPoint2Point(Vector3 point_new, KDTree kdTargetTree, float maxDistance, float[,] H, float[] b)
    {
        (Vector3 q, int index, float distSq) returnNearestSearch = kdTargetTree.NearestNeighbor(point_new);
        if (Mathf.Sqrt(returnNearestSearch.distSq) > maxDistance)
        {
            return (false, new float[,] { }, new float[] { }, new float[] { });
        }
        Vector3 qAssociated = returnNearestSearch.q;
        float[] errore = new float[] { point_new.x - qAssociated.x, point_new.y - qAssociated.y, point_new.z - qAssociated.z };
        float weight = (float)(1.0 / (1.0 + returnNearestSearch.distSq));

        (float[,] updatedH, float[] updatedB) accumulated = AccumulateHbP2P(point_new, errore, weight, H, b);
        return (true, accumulated.updatedH, accumulated.updatedB, errore);
    }

    private static (float[,], float[]) AccumulateHbP2P(Vector3 point_new, float[] errore, float w, float[,] H, float[] b)
    {
        float[,] jacobian = GetJacobianP2P(point_new);
        float[,] Hreturn = sumSquareMatrix6(H, productSquareMatrix6Scalar(multiply63by36(transposeMatrix36(jacobian),jacobian), w));
        float[] bReturn = sumVector6(b, productVector6Scalar(productMatrix63Vector3(transposeMatrix36(jacobian), errore), w));
        return (Hreturn, bReturn);
    }

    private static float[,] GetJacobianP2P(Vector3 point_new)
    {
        float[,] jacobian = Zeros(3, 6);
        jacobian[0, 0] = 1.0f;
        jacobian[1, 1] = 1.0f;
        jacobian[2, 2] = 1.0f;
        jacobian[0, 4] = point_new.z;
        jacobian[0, 5] = -1 * point_new.y;
        jacobian[1, 3] = -1 * point_new.z;
        jacobian[1, 5] = point_new.x;
        jacobian[2, 3] = point_new.y;
        jacobian[2, 4] = -1 * point_new.x;
        return jacobian;
    }

}
