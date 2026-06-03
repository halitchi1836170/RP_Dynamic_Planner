using System;
using UnityEngine;
using static MatrixVectorUtilities;

public class PoseMatrix4x4
{
    public static float[,] ExpMap(float[] delta, float thetaRotationTreshold = 1e-6f)
    {
        float[,] result = Identity4();
        if (delta.Length == 6)
        {
            float[] v = new float[3] { delta[0], delta[1], delta[2] };
            float[] w = new float[3] { delta[3], delta[4], delta[5] };
            float theta = getNormV3(w);

            if(theta < thetaRotationTreshold)
            {
                result[0, 3] = v[0];
                result[1, 3] = v[1];
                result[2, 3] = v[2];
            }
            else
            {
                float[,] wSkeyMatrix = getSkewSymmetricMatrix(w);
                float[,] wSquared = productSquareMatrix3(wSkeyMatrix, wSkeyMatrix);

                float[,] R = Identity3();
                R = sumSquareMatrix3(sumSquareMatrix3(R, productSquareMatrix3Scalar(wSkeyMatrix, Mathf.Sin(theta) / theta)), productSquareMatrix3Scalar(wSquared, (1 - Mathf.Cos(theta)) / (Mathf.Pow(theta, 2))));
                float[,] V = Identity3();
                V = sumSquareMatrix3(sumSquareMatrix3(V, productSquareMatrix3Scalar(wSkeyMatrix, (1 - Mathf.Cos(theta)) / (Mathf.Pow(theta, 2)))), productSquareMatrix3Scalar(wSquared, (theta - Mathf.Sin(theta)) / (Mathf.Pow(theta, 3))));
                float[] t = productSquareMatrix3Vector3(V, v);

                result[0, 0] = R[0, 0];
                result[0, 1] = R[0, 1];
                result[0, 2] = R[0, 2];

                result[1, 0] = R[1, 0];
                result[1, 1] = R[1, 1];
                result[1, 2] = R[1, 2];

                result[2, 0] = R[2, 0];
                result[2, 1] = R[2, 1];
                result[2, 2] = R[2, 2];

                result[0, 3] = t[0];
                result[1, 3] = t[1];
                result[2, 3] = t[2];
            }
        }
        else
        {
            Debug.Log("delta input vector has dimension <> (1,6), returning zero matrix");
        }
        return result;
    }

  
    public static Vector3 applyTransformation(float[, ] m, Vector3 point)
    {
        Vector3 result = new Vector3(); 
        if (m.GetLength(0)==4 && m.GetLength(1) == 4)
        {
            float[] position = new float[4] { point.x, point.y, point.z, 1 };
            float[] res = productSquareMatrix4Vector4(m, position);
            result.x = res[0];
            result.y = res[1];
            result.z = res[2];
        }
        else
        {
            Debug.Log("input matrix for apply trasnformation has no correct dimension, returning zero vector...");
        }
        return result;
    }


    public static float[] LogMap(float[,] T, float thetaRotationTreshold = 1e-6f)
    {
        // Rotation
        float r00 = T[0,0];
        float r01 = T[0,1];
        float r02 = T[0,2];
        float r10 = T[1,0];
        float r11 = T[1,1];
        float r12 = T[1,2];
        float r20 = T[2,0];
        float r21 = T[2,1];
        float r22 = T[2,2];
        // Translation
        float tx = T[0,3];
        float ty = T[1,3];
        float tz = T[2,3];

        // Trace
        float trace = r00 + r11 + r22;

        // Clamp for numerical stability
        float cosTheta = (trace - 1.0f) * 0.5f;
        cosTheta = MathF.Max(-1.0f, MathF.Min(1.0f, cosTheta));

        float theta = MathF.Acos(cosTheta);

        float rx, ry, rz;

        // Small angle approximation
        if (theta < thetaRotationTreshold)
        {
            rx = 0.5f * (r21 - r12);
            ry = 0.5f * (r02 - r20);
            rz = 0.5f * (r10 - r01);

            return new float[]
            {
                tx, ty, tz,
                rx, ry, rz
            };
        }

        float sinTheta = MathF.Sin(theta);

        float scale = theta / (2.0f * sinTheta);

        // so(3) logarithm
        rx = scale * (r21 - r12);
        ry = scale * (r02 - r20);
        rz = scale * (r10 - r01);

        // Skew matrix terms
        float wx = rx;
        float wy = ry;
        float wz = rz;

        float theta2 = theta * theta;

        float A = (1.0f - MathF.Cos(theta)) / theta2;
        float B = (theta - MathF.Sin(theta)) / (theta2 * theta);

        // W^2 components
        float wx2 = wx * wx;
        float wy2 = wy * wy;
        float wz2 = wz * wz;

        float wxy = wx * wy;
        float wxz = wx * wz;
        float wyz = wy * wz;

        // V matrix
        float v00 = 1 - B * (wy2 + wz2);
        float v01 = -A * wz + B * wxy;
        float v02 = A * wy + B * wxz;

        float v10 = A * wz + B * wxy;
        float v11 = 1 - B * (wx2 + wz2);
        float v12 = -A * wx + B * wyz;

        float v20 = -A * wy + B * wxz;
        float v21 = A * wx + B * wyz;
        float v22 = 1 - B * (wx2 + wy2);

        // Inverse of V
        float det =
            v00 * (v11 * v22 - v12 * v21) -
            v01 * (v10 * v22 - v12 * v20) +
            v02 * (v10 * v21 - v11 * v20);

        float invDet = 1.0f / det;

        float iv00 = (v11 * v22 - v12 * v21) * invDet;
        float iv01 = -(v01 * v22 - v02 * v21) * invDet;
        float iv02 = (v01 * v12 - v02 * v11) * invDet;

        float iv10 = -(v10 * v22 - v12 * v20) * invDet;
        float iv11 = (v00 * v22 - v02 * v20) * invDet;
        float iv12 = -(v00 * v12 - v02 * v10) * invDet;

        float iv20 = (v10 * v21 - v11 * v20) * invDet;
        float iv21 = -(v00 * v21 - v01 * v20) * invDet;
        float iv22 = (v00 * v11 - v01 * v10) * invDet;

        // rho = V^{-1} t
        float px = iv00 * tx + iv01 * ty + iv02 * tz;
        float py = iv10 * tx + iv11 * ty + iv12 * tz;
        float pz = iv20 * tx + iv21 * ty + iv22 * tz;

        return new float[]
        {
            px, py, pz,
            rx, ry, rz
        };
    }


    public static float[,] InverseT(float[,] T)
    {
        (float[,] R, float[] t) decomposedT = decomposeT(T);
        float[,] R_trasnpose = transposeSquareMatrix3(decomposedT.R);
        float[] newt = productVector3Scalar(productSquareMatrix3Vector3(R_trasnpose, decomposedT.t),-1.0f);
        float[,] composedT = composeT(R_trasnpose, newt);
        return composedT;
    }
}
