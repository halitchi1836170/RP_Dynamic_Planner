using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using System.Collections.Generic;
using System.Linq.Expressions;
using UnityEngine;

public class MatrixVectorUtilities
{

    public static float[,] Identity(int dimension)
    {
        float[,] result = Zeros(dimension, dimension);
        for (int r = 0; r < dimension; r++)
        {
            result[r, r] = 1.0f;
        }
        return result;
    }

    public static float[,] Identity3()
    {
        float[,] result = Zeros33();
        result[0, 0] = 1.0f;
        result[1, 1] = 1.0f;
        result[2, 2] = 1.0f;
        return result;
    }

    public static float[,] Identity4()
    {
        float[,] result = Zeros44();
        result[0, 0] = 1.0f;
        result[1, 1] = 1.0f;
        result[2, 2] = 1.0f;
        result[3, 3] = 1.0f;
        return result;
    }

    public static float[,] Identity6()
    {
        float[,] result = Zeros66();
        result[0, 0] = 1.0f;
        result[1, 1] = 1.0f;
        result[2, 2] = 1.0f;
        result[3, 3] = 1.0f;
        result[4, 4] = 1.0f;
        result[5, 5] = 1.0f;
        return result;
    }

    public static float[,] Zeros(int dr, int dc)
    {
        float[,] result = new float[dr, dc];
        return result;
    }

    public static float[] Zeros6() { 
        return new float[6]; 
    }

    public static float[,] Zeros33()
    {
        float[,] result = new float[3, 3];
        return result;
    }

    public static float[,] Zeros44()
    {
        float[,] result = new float[4, 4];
        return result;
    }

    public static float[,] Zeros66()
    {
        float[,] result = new float[6, 6];
        return result;
    }
    public static float getNorm(float[] vector)
    {
        float norm = 0.0f;
        if (vector.Length > 0)
        {
            for (int i = 0; i < vector.Length; i++)
            {
                norm = norm + Mathf.Pow((vector[i]), 2);
            }
        }
        return Mathf.Sqrt(norm);
    }

    public static float getNormV3(float[] vector)
    {
        float norm = 0.0f;
        if (vector.Length == 3)
        {
            norm = Mathf.Pow(vector[0], 2) + Mathf.Pow(vector[1], 2) + Mathf.Pow(vector[2], 2);
        }
        return Mathf.Sqrt(norm);
    }

    public static float[,] getSkewSymmetricMatrix(float[] vector)
    {
        float[,] result = Zeros(3, 3);
        if (vector.Length == 3)
        {
            result[0, 1] = -vector[2];
            result[0, 2] = vector[1];
            result[1, 0] = vector[2];
            result[1, 2] = -vector[0];
            result[2, 0] = -vector[1];
            result[2, 1] = vector[0];
        }
        else
        {
            Debug.Log("input vector has no dimension 3, returning zero matrix");
        }
        return result;
    }

    // Trasposta di una matrice 6x6.
    public static float[,] transposeSquareMatrix6(float[,] m)
    {
        float[,] result = new float[6, 6];
        for (int r = 0; r < 6; r++)
            for (int c = 0; c < 6; c++)
                result[c, r] = m[r, c];
        return result;
    }

    // Adjoint SE(3) (6x6) di una posa T=[[R,t],[0,1]] con ordinamento twist [traslazione, rotazione]:
    //   Adj(T) = [[ R,        [t]x * R ],
    //             [ 0(3x3),   R        ]]
    public static float[,] AdjointSE3(float[,] T)
    {
        float[,] R = getRFromT(T);
        float[] t = gettFromT(T);
        float[,] tSkewR = productSquareMatrix3(getSkewSymmetricMatrix(t), R);

        float[,] result = new float[6, 6];
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
            {
                result[r, c] = R[r, c];                // blocco alto-sinistra: R
                result[r, c + 3] = tSkewR[r, c];       // blocco alto-destra: [t]x R
                result[r + 3, c] = 0.0f;               // blocco basso-sinistra: 0
                result[r + 3, c + 3] = R[r, c];        // blocco basso-destra: R
            }
        }
        return result;
    }

    // "Little adjoint" ad_xi (6x6) del twist xi = [rho(0,1,2), omega(3,4,5)]:
    //   ad(xi) = [[ [omega]x, [rho]x ],
    //             [ 0(3x3),   [omega]x ]]
    public static float[,] littleAdjoint6(float[] xi)
    {
        float[] rho = new float[3] { xi[0], xi[1], xi[2] };
        float[] omega = new float[3] { xi[3], xi[4], xi[5] };
        float[,] rhoSkew = getSkewSymmetricMatrix(rho);
        float[,] omegaSkew = getSkewSymmetricMatrix(omega);

        float[,] result = new float[6, 6];
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
            {
                result[r, c] = omegaSkew[r, c];        // blocco alto-sinistra: [omega]x
                result[r, c + 3] = rhoSkew[r, c];      // blocco alto-destra: [rho]x
                result[r + 3, c] = 0.0f;               // blocco basso-sinistra: 0
                result[r + 3, c + 3] = omegaSkew[r, c];// blocco basso-destra: [omega]x
            }
        }
        return result;
    }

    public static float[,] sumSquareMatrix3(float[,] matrix1, float[,] matrix2)
    {
        float[,] result = Zeros33();
        if (matrix1.GetLength(0) == matrix2.GetLength(0) && matrix1.GetLength(1) == matrix2.GetLength(1) && matrix1.GetLength(1) == 3)
        {
            result[0, 0] = matrix1[0, 0] + matrix2[0, 0];
            result[0, 1] = matrix1[0, 1] + matrix2[0, 1];
            result[0, 2] = matrix1[0, 2] + matrix2[0, 2];

            result[1, 0] = matrix1[1, 0] + matrix2[1, 0];
            result[1, 1] = matrix1[1, 1] + matrix2[1, 1];
            result[1, 2] = matrix1[1, 2] + matrix2[1, 2];

            result[2, 0] = matrix1[2, 0] + matrix2[2, 0];
            result[2, 1] = matrix1[2, 1] + matrix2[2, 1];
            result[2, 2] = matrix1[2, 2] + matrix2[2, 2];
        }
        else
        {
            Debug.Log("matrixes has differente sizes, returning zero matrix");
        }
        return result;
    }

    public static float[,] sumSquareMatrix6(float[,] matrix1, float[,] matrix2)
    {
        float[,] result = Zeros66();
        if (matrix1.GetLength(0) == matrix2.GetLength(0) && matrix1.GetLength(1) == matrix2.GetLength(1) && matrix1.GetLength(1) == 6)
        {
            result[0, 0] = matrix1[0, 0] + matrix2[0, 0];
            result[0, 1] = matrix1[0, 1] + matrix2[0, 1];
            result[0, 2] = matrix1[0, 2] + matrix2[0, 2];
            result[0, 3] = matrix1[0, 3] + matrix2[0, 3];
            result[0, 4] = matrix1[0, 4] + matrix2[0, 4];
            result[0, 5] = matrix1[0, 5] + matrix2[0, 5];

            result[1, 0] = matrix1[1, 0] + matrix2[1, 0];
            result[1, 1] = matrix1[1, 1] + matrix2[1, 1];
            result[1, 2] = matrix1[1, 2] + matrix2[1, 2];
            result[1, 3] = matrix1[1, 3] + matrix2[1, 3];
            result[1, 4] = matrix1[1, 4] + matrix2[1, 4];
            result[1, 5] = matrix1[1, 5] + matrix2[1, 5];

            result[2, 0] = matrix1[2, 0] + matrix2[2, 0];
            result[2, 1] = matrix1[2, 1] + matrix2[2, 1];
            result[2, 2] = matrix1[2, 2] + matrix2[2, 2];
            result[2, 3] = matrix1[2, 3] + matrix2[2, 3];
            result[2, 4] = matrix1[2, 4] + matrix2[2, 4];
            result[2, 5] = matrix1[2, 5] + matrix2[2, 5];

            result[3, 0] = matrix1[3, 0] + matrix2[3, 0];
            result[3, 1] = matrix1[3, 1] + matrix2[3, 1];
            result[3, 2] = matrix1[3, 2] + matrix2[3, 2];
            result[3, 3] = matrix1[3, 3] + matrix2[3, 3];
            result[3, 4] = matrix1[3, 4] + matrix2[3, 4];
            result[3, 5] = matrix1[3, 5] + matrix2[3, 5];

            result[4, 0] = matrix1[4, 0] + matrix2[4, 0];
            result[4, 1] = matrix1[4, 1] + matrix2[4, 1];
            result[4, 2] = matrix1[4, 2] + matrix2[4, 2];
            result[4, 3] = matrix1[4, 3] + matrix2[4, 3];
            result[4, 4] = matrix1[4, 4] + matrix2[4, 4];
            result[4, 5] = matrix1[4, 5] + matrix2[4, 5];

            result[5, 0] = matrix1[5, 0] + matrix2[5, 0];
            result[5, 1] = matrix1[5, 1] + matrix2[5, 1];
            result[5, 2] = matrix1[5, 2] + matrix2[5, 2];
            result[5, 3] = matrix1[5, 3] + matrix2[5, 3];
            result[5, 4] = matrix1[5, 4] + matrix2[5, 4];
            result[5, 5] = matrix1[5, 5] + matrix2[5, 5];
        }
        else
        {
            Debug.Log("matrixes has differente sizes, returning zero matrix");
        }
        return result;
    }

    public static float[] sumVector6(float[] v1, float[] v2)
    {
        float[] result = Zeros6();
        if (v1.GetLength(0) == v2.GetLength(0)  && v2.GetLength(0) == 6)
        {
            result[0] = v1[0] + v2[ 0];
            result[1] = v1[1] + v2[ 1];
            result[2] = v1[2] + v2[ 2];
            result[3] = v1[3] + v2[ 3];
            result[4] = v1[4] + v2[ 4];
            result[5] = v1[5] + v2[ 5];
        }
        else
        {
            Debug.Log("vectors has differente sizes, returning zero vector");
        }
        return result;
    }

    public static Vector3 avgListVector3(List<Vector3> list)
    {
        Vector3 result = new Vector3();
        foreach(Vector3 v in list)
        {
            result += v;
        }
        result /= list.Count;
        return result;
    }

    public static float avgArrayFloats(float[] values)
    {
        float result = 0.0f;
        foreach(float v in values)
        {
            result += v;
        }
        return result / values.Length;
    }

    public static float[] CrossProductV3(float[] a, float[] b)
    {
        // Il prodotto vettoriale è definito solo nello spazio 3D
        return new float[]
        {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
        };
    }

    public static float DottProductV3(float[] a, float[] b)
    {
        // Il prodotto vettoriale è definito solo nello spazio 3D
        return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
    }

    public static float[,] getCovarianceMatrix3(List<Vector3> list)
    {
        Vector3 centroide = avgListVector3(list);
        float[,] result = Zeros33();
        foreach(Vector3 v in list)
        {
            Vector3 dif = (v - centroide);
            result = sumSquareMatrix3(result, productColumnRowVector3(dif,dif));
        }
        return result;
    }

    public static float[] getMinimumEigenvector(float[,] matrix)
    {
        DenseMatrix m = DenseMatrix.OfArray(matrix);
        var evd = m.Evd();
        Vector<System.Numerics.Complex> eigenvalues = evd.EigenValues;
        Matrix<float> eigenvectors = (Matrix<float>)evd.EigenVectors;

        float[] result = new float[3] { eigenvectors[0, eigenvalues.AbsoluteMinimumIndex()], eigenvectors[1, eigenvalues.AbsoluteMinimumIndex()] , eigenvectors[2, eigenvalues.AbsoluteMinimumIndex()] };
        return result;
    }

    public static float[,] productColumnRowVector3(Vector3 v1, Vector3 v2)
    {
        float[,] result = new float[3, 3];
        result[0, 0] = v1[0] * v2[0];
        result[0, 1] = v1[0] * v2[1];
        result[0, 2] = v1[0] * v2[2];

        result[1, 0] = v1[1] * v2[0];
        result[1, 1] = v1[1] * v2[1];
        result[1, 2] = v1[1] * v2[2];

        result[2, 0] = v1[2] * v2[0];
        result[2, 1] = v1[2] * v2[1];
        result[2, 2] = v1[2] * v2[2];
        return result;
    }

    public static float[,] productColumnRowVector6(float[] v1, float[] v2)
    {
        float[,] result = Zeros66();
        result[0, 0] = v1[0] * v2[0];
        result[0, 1] = v1[0] * v2[1];
        result[0, 2] = v1[0] * v2[2];
        result[0, 3] = v1[0] * v2[3];
        result[0, 4] = v1[0] * v2[4];
        result[0, 5] = v1[0] * v2[5];

        result[1, 0] = v1[1] * v2[0];
        result[1, 1] = v1[1] * v2[1];
        result[1, 2] = v1[1] * v2[2];
        result[1, 3] = v1[1] * v2[3];
        result[1, 4] = v1[1] * v2[4];
        result[1, 5] = v1[1] * v2[5];

        result[2, 0] = v1[2] * v2[0];
        result[2, 1] = v1[2] * v2[1];
        result[2, 2] = v1[2] * v2[2];
        result[2, 3] = v1[2] * v2[3];
        result[2, 4] = v1[2] * v2[4];
        result[2, 5] = v1[2] * v2[5];

        result[3, 0] = v1[3] * v2[0];
        result[3, 1] = v1[3] * v2[1];
        result[3, 2] = v1[3] * v2[2];
        result[3, 3] = v1[3] * v2[3];
        result[3, 4] = v1[3] * v2[4];
        result[3, 5] = v1[3] * v2[5];

        result[4, 0] = v1[4] * v2[0];
        result[4, 1] = v1[4] * v2[1];
        result[4, 2] = v1[4] * v2[2];
        result[4, 3] = v1[4] * v2[3];
        result[4, 4] = v1[4] * v2[4];
        result[4, 5] = v1[4] * v2[5];

        result[5, 0] = v1[5] * v2[0];
        result[5, 1] = v1[5] * v2[1];
        result[5, 2] = v1[5] * v2[2];
        result[5, 3] = v1[5] * v2[3];
        result[5, 4] = v1[5] * v2[4];
        result[5, 5] = v1[5] * v2[5];
        return result;
    }

    public static float[,] productSquareMatrix3(float[,] m1, float[,] m2)
    {
        float[,] result = Zeros33();
        if (m1.GetLength(0) == m2.GetLength(0) && m1.GetLength(1) == m2.GetLength(1) && m1.GetLength(1) == 3)
        {
            result[0, 0] = m1[0, 0] * m2[0, 0] + m1[0, 1] * m2[1, 0] + m1[0, 2] * m2[2, 0];
            result[0, 1] = m1[0, 0] * m2[0, 1] + m1[0, 1] * m2[1, 1] + m1[0, 2] * m2[2, 1];
            result[0, 2] = m1[0, 0] * m2[0, 2] + m1[0, 1] * m2[1, 2] + m1[0, 2] * m2[2, 2];

            result[1, 0] = m1[1, 0] * m2[0, 0] + m1[1, 1] * m2[1, 0] + m1[1, 2] * m2[2, 0];
            result[1, 1] = m1[1, 0] * m2[0, 1] + m1[1, 1] * m2[1, 1] + m1[1, 2] * m2[2, 1];
            result[1, 2] = m1[1, 0] * m2[0, 2] + m1[1, 1] * m2[1, 2] + m1[1, 2] * m2[2, 2];

            result[2, 0] = m1[2, 0] * m2[0, 0] + m1[2, 1] * m2[1, 0] + m1[2, 2] * m2[2, 0];
            result[2, 1] = m1[2, 0] * m2[0, 1] + m1[2, 1] * m2[1, 1] + m1[2, 2] * m2[2, 1];
            result[2, 2] = m1[2, 0] * m2[0, 2] + m1[2, 1] * m2[1, 2] + m1[2, 2] * m2[2, 2];
        }
        else
        {
            Debug.Log("matrixes has differente sizes, returning zero matrix");
        }
        return result;
    }

    public static float[,] productSquareMatrix4(float[,] m1, float[,] m2)
    {
        float[,] result = Zeros44();
        if (m1.GetLength(0) == m2.GetLength(0) && m1.GetLength(1) == m2.GetLength(1) && m1.GetLength(1) == 4)
        {
            result[0, 0] = m1[0, 0] * m2[0, 0] + m1[0, 1] * m2[1, 0] + m1[0, 2] * m2[2, 0] + m1[0, 3] * m2[3, 0];
            result[0, 1] = m1[0, 0] * m2[0, 1] + m1[0, 1] * m2[1, 1] + m1[0, 2] * m2[2, 1] + m1[0, 3] * m2[3, 1];
            result[0, 2] = m1[0, 0] * m2[0, 2] + m1[0, 1] * m2[1, 2] + m1[0, 2] * m2[2, 2] + m1[0, 3] * m2[3, 2];
            result[0, 3] = m1[0, 0] * m2[0, 3] + m1[0, 1] * m2[1, 3] + m1[0, 2] * m2[2, 3] + m1[0, 3] * m2[3, 3];

            result[1, 0] = m1[1, 0] * m2[0, 0] + m1[1, 1] * m2[1, 0] + m1[1, 2] * m2[2, 0] + m1[1, 3] * m2[3, 0];
            result[1, 1] = m1[1, 0] * m2[0, 1] + m1[1, 1] * m2[1, 1] + m1[1, 2] * m2[2, 1] + m1[1, 3] * m2[3, 1];
            result[1, 2] = m1[1, 0] * m2[0, 2] + m1[1, 1] * m2[1, 2] + m1[1, 2] * m2[2, 2] + m1[1, 3] * m2[3, 2];
            result[1, 3] = m1[1, 0] * m2[0, 3] + m1[1, 1] * m2[1, 3] + m1[1, 2] * m2[2, 3] + m1[1, 3] * m2[3, 3];

            result[2, 0] = m1[2, 0] * m2[0, 0] + m1[2, 1] * m2[1, 0] + m1[2, 2] * m2[2, 0] + m1[2, 3] * m2[3, 0];
            result[2, 1] = m1[2, 0] * m2[0, 1] + m1[2, 1] * m2[1, 1] + m1[2, 2] * m2[2, 1] + m1[2, 3] * m2[3, 1];
            result[2, 2] = m1[2, 0] * m2[0, 2] + m1[2, 1] * m2[1, 2] + m1[2, 2] * m2[2, 2] + m1[2, 3] * m2[3, 2];
            result[2, 3] = m1[2, 0] * m2[0, 3] + m1[2, 1] * m2[1, 3] + m1[2, 2] * m2[2, 3] + m1[2, 3] * m2[3, 3];

            result[3, 0] = m1[3, 0] * m2[0, 0] + m1[3, 1] * m2[1, 0] + m1[3, 2] * m2[2, 0] + m1[3, 3] * m2[3, 0];
            result[3, 1] = m1[3, 0] * m2[0, 1] + m1[3, 1] * m2[1, 1] + m1[3, 2] * m2[2, 1] + m1[3, 3] * m2[3, 1];
            result[3, 2] = m1[3, 0] * m2[0, 2] + m1[3, 1] * m2[1, 2] + m1[3, 2] * m2[2, 2] + m1[3, 3] * m2[3, 2];
            result[3, 3] = m1[3, 0] * m2[0, 3] + m1[3, 1] * m2[1, 3] + m1[3, 2] * m2[2, 3] + m1[3, 3] * m2[3, 3];
        }
        else
        {
            Debug.Log("matrixes has differente sizes, returning zero matrix");
        }
        return result;
    }

    public static float[,] productSquareMatrix6(float[,] m1, float[,] m2)
    {
        float[,] result = Zeros66();
        if (m1.GetLength(0) == m2.GetLength(0) && m1.GetLength(1) == m2.GetLength(1) && m1.GetLength(1) == 6)
        {
            result[0, 0] = m1[0, 0] * m2[0, 0] + m1[0, 1] * m2[1, 0] + m1[0, 2] * m2[2, 0] + m1[0, 3] * m2[3, 0] + m1[0, 4] * m2[4, 0] + m1[0, 5] * m2[5, 0];
            result[0, 1] = m1[0, 0] * m2[0, 1] + m1[0, 1] * m2[1, 1] + m1[0, 2] * m2[2, 1] + m1[0, 3] * m2[3, 1] + m1[0, 4] * m2[4, 1] + m1[0, 5] * m2[5, 1];
            result[0, 2] = m1[0, 0] * m2[0, 2] + m1[0, 1] * m2[1, 2] + m1[0, 2] * m2[2, 2] + m1[0, 3] * m2[3, 2] + m1[0, 4] * m2[4, 2] + m1[0, 5] * m2[5, 2];
            result[0, 3] = m1[0, 0] * m2[0, 3] + m1[0, 1] * m2[1, 3] + m1[0, 2] * m2[2, 3] + m1[0, 3] * m2[3, 3] + m1[0, 4] * m2[4, 3] + m1[0, 5] * m2[5, 3];
            result[0, 4] = m1[0, 0] * m2[0, 4] + m1[0, 1] * m2[1, 4] + m1[0, 2] * m2[2, 4] + m1[0, 3] * m2[3, 4] + m1[0, 4] * m2[4, 4] + m1[0, 5] * m2[5, 4];
            result[0, 5] = m1[0, 0] * m2[0, 5] + m1[0, 1] * m2[1, 5] + m1[0, 2] * m2[2, 5] + m1[0, 3] * m2[3, 5] + m1[0, 4] * m2[4, 5] + m1[0, 5] * m2[5, 5];

            result[1, 0] = m1[1, 0] * m2[0, 0] + m1[1, 1] * m2[1, 0] + m1[1, 2] * m2[2, 0] + m1[1, 3] * m2[3, 0] + m1[1, 4] * m2[4, 0] + m1[1, 5] * m2[5, 0];
            result[1, 1] = m1[1, 0] * m2[0, 1] + m1[1, 1] * m2[1, 1] + m1[1, 2] * m2[2, 1] + m1[1, 3] * m2[3, 1] + m1[1, 4] * m2[4, 1] + m1[1, 5] * m2[5, 1];
            result[1, 2] = m1[1, 0] * m2[0, 2] + m1[1, 1] * m2[1, 2] + m1[1, 2] * m2[2, 2] + m1[1, 3] * m2[3, 2] + m1[1, 4] * m2[4, 2] + m1[1, 5] * m2[5, 2];
            result[1, 3] = m1[1, 0] * m2[0, 3] + m1[1, 1] * m2[1, 3] + m1[1, 2] * m2[2, 3] + m1[1, 3] * m2[3, 3] + m1[1, 4] * m2[4, 3] + m1[1, 5] * m2[5, 3];
            result[1, 4] = m1[1, 0] * m2[0, 4] + m1[1, 1] * m2[1, 4] + m1[1, 2] * m2[2, 4] + m1[1, 3] * m2[3, 4] + m1[1, 4] * m2[4, 4] + m1[1, 5] * m2[5, 4];
            result[1, 5] = m1[1, 0] * m2[0, 5] + m1[1, 1] * m2[1, 5] + m1[1, 2] * m2[2, 5] + m1[1, 3] * m2[3, 5] + m1[1, 4] * m2[4, 5] + m1[1, 5] * m2[5, 5];

            result[2, 0] = m1[2, 0] * m2[0, 0] + m1[2, 1] * m2[1, 0] + m1[2, 2] * m2[2, 0] + m1[2, 3] * m2[3, 0] + m1[2, 4] * m2[4, 0] + m1[2, 5] * m2[5, 0];
            result[2, 1] = m1[2, 0] * m2[0, 1] + m1[2, 1] * m2[1, 1] + m1[2, 2] * m2[2, 1] + m1[2, 3] * m2[3, 1] + m1[2, 4] * m2[4, 1] + m1[2, 5] * m2[5, 1];
            result[2, 2] = m1[2, 0] * m2[0, 2] + m1[2, 1] * m2[1, 2] + m1[2, 2] * m2[2, 2] + m1[2, 3] * m2[3, 2] + m1[2, 4] * m2[4, 2] + m1[2, 5] * m2[5, 2];
            result[2, 3] = m1[2, 0] * m2[0, 3] + m1[2, 1] * m2[1, 3] + m1[2, 2] * m2[2, 3] + m1[2, 3] * m2[3, 3] + m1[2, 4] * m2[4, 3] + m1[2, 5] * m2[5, 3];
            result[2, 4] = m1[2, 0] * m2[0, 4] + m1[2, 1] * m2[1, 4] + m1[2, 2] * m2[2, 4] + m1[2, 3] * m2[3, 4] + m1[2, 4] * m2[4, 4] + m1[2, 5] * m2[5, 4];
            result[2, 5] = m1[2, 0] * m2[0, 5] + m1[2, 1] * m2[1, 5] + m1[2, 2] * m2[2, 5] + m1[2, 3] * m2[3, 5] + m1[2, 4] * m2[4, 5] + m1[2, 5] * m2[5, 5];

            result[3, 0] = m1[3, 0] * m2[0, 0] + m1[3, 1] * m2[1, 0] + m1[3, 2] * m2[2, 0] + m1[3, 3] * m2[3, 0] + m1[3, 4] * m2[4, 0] + m1[3, 5] * m2[5, 0];
            result[3, 1] = m1[3, 0] * m2[0, 1] + m1[3, 1] * m2[1, 1] + m1[3, 2] * m2[2, 1] + m1[3, 3] * m2[3, 1] + m1[3, 4] * m2[4, 1] + m1[3, 5] * m2[5, 1];
            result[3, 2] = m1[3, 0] * m2[0, 2] + m1[3, 1] * m2[1, 2] + m1[3, 2] * m2[2, 2] + m1[3, 3] * m2[3, 2] + m1[3, 4] * m2[4, 2] + m1[3, 5] * m2[5, 2];
            result[3, 3] = m1[3, 0] * m2[0, 3] + m1[3, 1] * m2[1, 3] + m1[3, 2] * m2[2, 3] + m1[3, 3] * m2[3, 3] + m1[3, 4] * m2[4, 3] + m1[3, 5] * m2[5, 3];
            result[3, 4] = m1[3, 0] * m2[0, 4] + m1[3, 1] * m2[1, 4] + m1[3, 2] * m2[2, 4] + m1[3, 3] * m2[3, 4] + m1[3, 4] * m2[4, 4] + m1[3, 5] * m2[5, 4];
            result[3, 5] = m1[3, 0] * m2[0, 5] + m1[3, 1] * m2[1, 5] + m1[3, 2] * m2[2, 5] + m1[3, 3] * m2[3, 5] + m1[3, 4] * m2[4, 5] + m1[3, 5] * m2[5, 5];

            result[4, 0] = m1[4, 0] * m2[0, 0] + m1[4, 1] * m2[1, 0] + m1[4, 2] * m2[2, 0] + m1[4, 3] * m2[3, 0] + m1[4, 4] * m2[4, 0] + m1[4, 5] * m2[5, 0];
            result[4, 1] = m1[4, 0] * m2[0, 1] + m1[4, 1] * m2[1, 1] + m1[4, 2] * m2[2, 1] + m1[4, 3] * m2[3, 1] + m1[4, 4] * m2[4, 1] + m1[4, 5] * m2[5, 1];
            result[4, 2] = m1[4, 0] * m2[0, 2] + m1[4, 1] * m2[1, 2] + m1[4, 2] * m2[2, 2] + m1[4, 3] * m2[3, 2] + m1[4, 4] * m2[4, 2] + m1[4, 5] * m2[5, 2];
            result[4, 3] = m1[4, 0] * m2[0, 3] + m1[4, 1] * m2[1, 3] + m1[4, 2] * m2[2, 3] + m1[4, 3] * m2[3, 3] + m1[4, 4] * m2[4, 3] + m1[4, 5] * m2[5, 3];
            result[4, 4] = m1[4, 0] * m2[0, 4] + m1[4, 1] * m2[1, 4] + m1[4, 2] * m2[2, 4] + m1[4, 3] * m2[3, 4] + m1[4, 4] * m2[4, 4] + m1[4, 5] * m2[5, 4];
            result[4, 5] = m1[4, 0] * m2[0, 5] + m1[4, 1] * m2[1, 5] + m1[4, 2] * m2[2, 5] + m1[4, 3] * m2[3, 5] + m1[4, 4] * m2[4, 5] + m1[4, 5] * m2[5, 5];

            result[5, 0] = m1[5, 0] * m2[0, 0] + m1[5, 1] * m2[1, 0] + m1[5, 2] * m2[2, 0] + m1[5, 3] * m2[3, 0] + m1[5, 4] * m2[4, 0] + m1[5, 5] * m2[5, 0];
            result[5, 1] = m1[5, 0] * m2[0, 1] + m1[5, 1] * m2[1, 1] + m1[5, 2] * m2[2, 1] + m1[5, 3] * m2[3, 1] + m1[5, 4] * m2[4, 1] + m1[5, 5] * m2[5, 1];
            result[5, 2] = m1[5, 0] * m2[0, 2] + m1[5, 1] * m2[1, 2] + m1[5, 2] * m2[2, 2] + m1[5, 3] * m2[3, 2] + m1[5, 4] * m2[4, 2] + m1[5, 5] * m2[5, 2];
            result[5, 3] = m1[5, 0] * m2[0, 3] + m1[5, 1] * m2[1, 3] + m1[5, 2] * m2[2, 3] + m1[5, 3] * m2[3, 3] + m1[5, 4] * m2[4, 3] + m1[5, 5] * m2[5, 3];
            result[5, 4] = m1[5, 0] * m2[0, 4] + m1[5, 1] * m2[1, 4] + m1[5, 2] * m2[2, 4] + m1[5, 3] * m2[3, 4] + m1[5, 4] * m2[4, 4] + m1[5, 5] * m2[5, 4];
            result[5, 5] = m1[5, 0] * m2[0, 5] + m1[5, 1] * m2[1, 5] + m1[5, 2] * m2[2, 5] + m1[5, 3] * m2[3, 5] + m1[5, 4] * m2[4, 5] + m1[5, 5] * m2[5, 5];
        }
        else
        {
            Debug.Log("matrixes has differente sizes, returning zero matrix");
        }
        return result;
    }

    public static float[] productSquareMatrix3Vector3(float[,] m, float[] v)
    {
        float[] result = new float[3];
        if (m.GetLength(1) == v.Length && m.GetLength(1) == 3 && m.GetLength(0) == 3)
        {
            result[0] = m[0, 0] * v[0] + m[0, 1] * v[1] + m[0, 2] * v[2];
            result[1] = m[1, 0] * v[0] + m[1, 1] * v[1] + m[1, 2] * v[2];
            result[2] = m[2, 0] * v[0] + m[2, 1] * v[1] + m[2, 2] * v[2];
        }
        else
        {
            Debug.Log("Incorrect dimension for matrix vector multiplication, returning zero vector");
        }
        return result;
    }

    public static float[] productSquareMatrix4Vector4(float[,] m, float[] v)
    {
        float[] result = new float[4];
        if (m.GetLength(1) == v.Length && m.GetLength(1) == 4 && m.GetLength(0) == 4)
        {
            result[0] = m[0, 0] * v[0] + m[0, 1] * v[1] + m[0, 2] * v[2] + m[0, 3] * v[3];
            result[1] = m[1, 0] * v[0] + m[1, 1] * v[1] + m[1, 2] * v[2] + m[1, 3] * v[3];
            result[2] = m[2, 0] * v[0] + m[2, 1] * v[1] + m[2, 2] * v[2] + m[2, 3] * v[3];
            result[3] = m[3, 0] * v[0] + m[3, 1] * v[1] + m[3, 2] * v[2] + m[3, 3] * v[3];
        }
        else
        {
            Debug.Log("Incorrect dimension for matrix vector multiplication, returning zero vector");
        }
        return result;
    }

    public static float[] productSquareMatrix6Vector6(float[,] m, float[] v)
    {
        float[] result = new float[6];
        if (m.GetLength(1) == v.Length && m.GetLength(1) == 6 && m.GetLength(0) == 6)
        {
            result[0] = m[0, 0] * v[0] + m[0, 1] * v[1] + m[0, 2] * v[2] + m[0, 3] * v[3] + m[0, 4] * v[4] + m[0, 5] * v[5];
            result[1] = m[1, 0] * v[0] + m[1, 1] * v[1] + m[1, 2] * v[2] + m[1, 3] * v[3] + m[1, 4] * v[4] + m[1, 5] * v[5];
            result[2] = m[2, 0] * v[0] + m[2, 1] * v[1] + m[2, 2] * v[2] + m[2, 3] * v[3] + m[2, 4] * v[4] + m[2, 5] * v[5];
            result[3] = m[3, 0] * v[0] + m[3, 1] * v[1] + m[3, 2] * v[2] + m[3, 3] * v[3] + m[3, 4] * v[4] + m[3, 5] * v[5];
            result[4] = m[4, 0] * v[0] + m[4, 1] * v[1] + m[4, 2] * v[2] + m[4, 3] * v[3] + m[4, 4] * v[4] + m[4, 5] * v[5];
            result[5] = m[5, 0] * v[0] + m[5, 1] * v[1] + m[5, 2] * v[2] + m[5, 3] * v[3] + m[5, 4] * v[4] + m[5, 5] * v[5];
        }
        else
        {
            Debug.Log("Incorrect dimension for matrix vector multiplication, returning zero vector");
        }
        return result;
    }

    public static float[] productMatrix63Vector3(float[,] m, float[] v)
    {
        float[] result = new float[6];
        if (m.GetLength(1) == v.Length && m.GetLength(1) == 3 && m.GetLength(0) == 6)
        {
            result[0] = m[0, 0] * v[0] + m[0, 1] * v[1] + m[0, 2] * v[2];
            result[1] = m[1, 0] * v[0] + m[1, 1] * v[1] + m[1, 2] * v[2];
            result[2] = m[2, 0] * v[0] + m[2, 1] * v[1] + m[2, 2] * v[2];
            result[3] = m[3, 0] * v[0] + m[3, 1] * v[1] + m[3, 2] * v[2];
            result[4] = m[4, 0] * v[0] + m[4, 1] * v[1] + m[4, 2] * v[2];
            result[5] = m[5, 0] * v[0] + m[5, 1] * v[1] + m[5, 2] * v[2];

        }
        else
        {
            Debug.Log("Incorrect dimension for matrix vector multiplication, returning zero vector");
        }
        return result;
    }

    public static float[,] productSquareMatrix3Scalar(float[,] m, float scalar)
    {
        float[,] result = Zeros33();
        result[0, 0] = m[0, 0] * scalar;
        result[0, 1] = m[0, 1] * scalar;
        result[0, 2] = m[0, 2] * scalar;

        result[1, 0] = m[1, 0] * scalar;
        result[1, 1] = m[1, 1] * scalar;
        result[1, 2] = m[1, 2] * scalar;

        result[2, 0] = m[2, 0] * scalar;
        result[2, 1] = m[2, 1] * scalar;
        result[2, 2] = m[2, 2] * scalar;
        return result;
    }

    public static float[,] productSquareMatrix4Scalar(float[,] m, float scalar)
    {
        float[,] result = Zeros44();
        result[0, 0] = m[0, 0] * scalar;
        result[0, 1] = m[0, 1] * scalar;
        result[0, 2] = m[0, 2] * scalar;
        result[0, 3] = m[0, 3] * scalar;

        result[1, 0] = m[1, 0] * scalar;
        result[1, 1] = m[1, 1] * scalar;
        result[1, 2] = m[1, 2] * scalar;
        result[1, 3] = m[1, 3] * scalar;

        result[2, 0] = m[2, 0] * scalar;
        result[2, 1] = m[2, 1] * scalar;
        result[2, 2] = m[2, 2] * scalar;
        result[2, 3] = m[2, 3] * scalar;

        result[3, 0] = m[3, 0] * scalar;
        result[3, 1] = m[3, 1] * scalar;
        result[3, 2] = m[3, 2] * scalar;
        result[3, 3] = m[3, 3] * scalar;

        return result;
    }

    public static float[,] productSquareMatrix6Scalar(float[,] m, float scalar)
    {
        float[,] result = Zeros66();
        result[0, 0] = m[0, 0] * scalar;
        result[0, 1] = m[0, 1] * scalar;
        result[0, 2] = m[0, 2] * scalar;
        result[0, 3] = m[0, 3] * scalar;
        result[0, 4] = m[0, 4] * scalar;
        result[0, 5] = m[0, 5] * scalar;

        result[1, 0] = m[1, 0] * scalar;
        result[1, 1] = m[1, 1] * scalar;
        result[1, 2] = m[1, 2] * scalar;
        result[1, 3] = m[1, 3] * scalar;
        result[1, 4] = m[1, 4] * scalar;
        result[1, 5] = m[1, 5] * scalar;

        result[2, 0] = m[2, 0] * scalar;
        result[2, 1] = m[2, 1] * scalar;
        result[2, 2] = m[2, 2] * scalar;
        result[2, 3] = m[2, 3] * scalar;
        result[2, 4] = m[2, 4] * scalar;
        result[2, 5] = m[2, 5] * scalar;

        result[3, 0] = m[3, 0] * scalar;
        result[3, 1] = m[3, 1] * scalar;
        result[3, 2] = m[3, 2] * scalar;
        result[3, 3] = m[3, 3] * scalar;
        result[3, 4] = m[3, 4] * scalar;
        result[3, 5] = m[3, 5] * scalar;

        result[4, 0] = m[4, 0] * scalar;
        result[4, 1] = m[4, 1] * scalar;
        result[4, 2] = m[4, 2] * scalar;
        result[4, 3] = m[4, 3] * scalar;
        result[4, 4] = m[4, 4] * scalar;
        result[4, 5] = m[4, 5] * scalar;

        result[5, 0] = m[5, 0] * scalar;
        result[5, 1] = m[5, 1] * scalar;
        result[5, 2] = m[5, 2] * scalar;
        result[5, 3] = m[5, 3] * scalar;
        result[5, 4] = m[5, 4] * scalar;
        result[5, 5] = m[5, 5] * scalar;

        return result;
    }

    public static float[] productVector6Scalar(float[] v, float scalar)
    {
        float[] result = Zeros6();
        result[0] = v[0] * scalar;
        result[1] = v[1] * scalar;
        result[2] = v[2] * scalar;
        result[3] = v[3] * scalar;
        result[4] = v[4] * scalar;
        result[5] = v[5] * scalar;

        return result;
    }

    public static float[] productVector3Scalar(float[] v, float scalar)
    {
        float[] result = new float[3];
        result[0] = v[0] * scalar;
        result[1] = v[1] * scalar;
        result[2] = v[2] * scalar;

        return result;
    }

    public static float[,] transposeMatrix36(float[,] m)
    {
        float[,] mReturn = new float[6,3];
        if(m.GetLength(0)==3 && m.GetLength(1) == 6)
        {
            // Riga 0 di m diventa Colonna 0 di mReturn
            mReturn[0, 0] = m[0, 0];
            mReturn[1, 0] = m[0, 1];
            mReturn[2, 0] = m[0, 2];
            mReturn[3, 0] = m[0, 3];
            mReturn[4, 0] = m[0, 4];
            mReturn[5, 0] = m[0, 5];

            // Riga 1 di m diventa Colonna 1 di mReturn
            mReturn[0, 1] = m[1, 0];
            mReturn[1, 1] = m[1, 1];
            mReturn[2, 1] = m[1, 2];
            mReturn[3, 1] = m[1, 3];
            mReturn[4, 1] = m[1, 4];
            mReturn[5, 1] = m[1, 5];

            // Riga 2 di m diventa Colonna 2 di mReturn
            mReturn[0, 2] = m[2, 0];
            mReturn[1, 2] = m[2, 1];
            mReturn[2, 2] = m[2, 2];
            mReturn[3, 2] = m[2, 3];
            mReturn[4, 2] = m[2, 4];
            mReturn[5, 2] = m[2, 5];
        }
        else
        {
            Debug.Log("Error on input matrix for transposition...");
        }
        return mReturn;
    }

    public static float[,] transposeSquareMatrix3(float[,] m)
    {
        float[,] mReturn = new float[3, 3];
        if (m.GetLength(0) == 3 && m.GetLength(1) == 3)
        {
            // Riga 0 di m diventa Colonna 0 di mReturn
            mReturn[0, 0] = m[0, 0];
            mReturn[1, 0] = m[0, 1];
            mReturn[2, 0] = m[0, 2];

            // Riga 1 di m diventa Colonna 1 di mReturn
            mReturn[0, 1] = m[1, 0];
            mReturn[1, 1] = m[1, 1];
            mReturn[2, 1] = m[1, 2];

            // Riga 2 di m diventa Colonna 2 di mReturn
            mReturn[0, 2] = m[2, 0];
            mReturn[1, 2] = m[2, 1];
            mReturn[2, 2] = m[2, 2];
        }
        else
        {
            Debug.Log("Error on input matrix for transposition...");
        }
        return mReturn;
    }

    public static (float[,], float[]) decomposeT(float[,] T)
    {
        float[,] R = getRFromT(T);
        float[] t = gettFromT(T);
        return (R, t);
    }

    public static float[] getSub6Vector(float[] v, int index)
    {
        float[] result = new float[6];
        if(v.Length >= index + 6)
        {
            result[0] = v[index];
            result[1] = v[index+1];
            result[2] = v[index+2];
            result[3] = v[index+3];
            result[4] = v[index+4];
            result[5] = v[index+5];
        }
        return result;
    }

    public static float[,] composeRByColumnVectors(float[] c1, float[] c2, float[] c3)
    {
        float[,] result = new float[3, 3];
        if(c1.Length==3 && c2.Length==3 && c3.Length == 3)
        {
            result[0, 0] = c1[0];
            result[1, 0] = c1[1];
            result[2, 0] = c1[2];

            result[0, 1] = c2[0];
            result[1, 1] = c2[1];
            result[2, 1] = c2[2];

            result[0, 2] = c3[0];
            result[1, 2] = c3[1];
            result[2, 2] = c3[2];
        }
        return result;
    }

    // Proietta una posa SE(3) sul piano orizzontale (Y = verticale in Unity):
    // azzera la traslazione verticale e livella la rotazione a solo-yaw attorno a Y.
    // Usata sia per l'odometria (TWorldICP) sia per le misure di loop closure, così TUTTI
    // gli edge del grafo restano planari e coerenti tra loro.
    public static float[,] projectPoseToPlane(float[,] T)
    {
        float tx = T[0, 3];
        float tz = T[2, 3];

        // forward = terza colonna (asse Z locale), proiettato sull'orizzontale
        float[] fwd_h = new float[3] { T[0, 2], 0f, T[2, 2] };
        float fwdNorm = getNormV3(fwd_h);
        if (fwdNorm < 1e-6f)
        {
            // forward quasi verticale (degenere): lascia la rotazione, azzera solo la Y
            float[,] degen = (float[,])T.Clone();
            degen[1, 3] = 0f;
            return degen;
        }
        fwd_h = productVector3Scalar(fwd_h, 1f / fwdNorm);

        float[] up = new float[3] { 0f, 1f, 0f };
        float[] rightRaw = CrossProductV3(up, fwd_h);
        float[] right = productVector3Scalar(rightRaw, 1f / getNormV3(rightRaw));

        float[,] R = composeRByColumnVectors(right, up, fwd_h);
        return composeT(R, new float[3] { tx, 0f, tz });
    }

    public static float[,] composeT(float[,] R, float[] t)
    {
        float[,] T = new float[4,4];
        if(R.GetLength(0)==3 && R.GetLength(1)==3 && t.Length == 3)
        {
            T[0, 0] = R[0, 0];
            T[0, 1] = R[0, 1];
            T[0, 2] = R[0, 2];

            T[1, 0] = R[1, 0];
            T[1, 1] = R[1, 1];
            T[1, 2] = R[1, 2];

            T[2, 0] = R[2, 0];
            T[2, 1] = R[2, 1];
            T[2, 2] = R[2, 2];

            T[0, 3] = t[0];
            T[1, 3] = t[1];
            T[2, 3] = t[2];

            T[3, 0] = 0.0f;
            T[3, 1] = 0.0f;
            T[3, 2] = 0.0f;
            T[3, 3] = 1.0f;
        }
        return T;
    }

    public static float[,] getRFromT(float[,] T)
    {
        float[,] returnR = new float[3, 3];
        if(T.GetLength(0) == 4 && T.GetLength(1) == 4)
        {
            returnR[0, 0] = T[0, 0];
            returnR[0, 1] = T[0, 1];
            returnR[0, 2] = T[0, 2];

            returnR[1, 0] = T[1, 0];
            returnR[1, 1] = T[1, 1];
            returnR[1, 2] = T[1, 2];

            returnR[2, 0] = T[2, 0];
            returnR[2, 1] = T[2, 1];
            returnR[2, 2] = T[2, 2];
        }
        return returnR;
    }

    public static float[] gettFromT(float[,] T)
    {
        float[] returnt = new float[3];
        if (T.GetLength(0) == 4 && T.GetLength(1) == 4)
        {
            returnt[0] = T[0, 3];
            returnt[1] = T[1, 3];
            returnt[2] = T[2, 3];
        }
        return returnt;
    }

    public static float[,] multiply36by63(float[,] A, float[,] B)
    {
        // Risultato di (3x6) * (6x3) è una matrice 3x3
        float[,] R = new float[3, 3];

        if (A.GetLength(0) == 3 && A.GetLength(1) == 6 && B.GetLength(0) == 6 && B.GetLength(1) == 3)
        {
            // --- RIGA 0 ---
            R[0, 0] = A[0, 0] * B[0, 0] + A[0, 1] * B[1, 0] + A[0, 2] * B[2, 0] + A[0, 3] * B[3, 0] + A[0, 4] * B[4, 0] + A[0, 5] * B[5, 0];
            R[0, 1] = A[0, 0] * B[0, 1] + A[0, 1] * B[1, 1] + A[0, 2] * B[2, 1] + A[0, 3] * B[3, 1] + A[0, 4] * B[4, 1] + A[0, 5] * B[5, 1];
            R[0, 2] = A[0, 0] * B[0, 2] + A[0, 1] * B[1, 2] + A[0, 2] * B[2, 2] + A[0, 3] * B[3, 2] + A[0, 4] * B[4, 2] + A[0, 5] * B[5, 2];

            // --- RIGA 1 ---
            R[1, 0] = A[1, 0] * B[0, 0] + A[1, 1] * B[1, 0] + A[1, 2] * B[2, 0] + A[1, 3] * B[3, 0] + A[1, 4] * B[4, 0] + A[1, 5] * B[5, 0];
            R[1, 1] = A[1, 0] * B[0, 1] + A[1, 1] * B[1, 1] + A[1, 2] * B[2, 1] + A[1, 3] * B[3, 1] + A[1, 4] * B[4, 1] + A[1, 5] * B[5, 1];
            R[1, 2] = A[1, 0] * B[0, 2] + A[1, 1] * B[1, 2] + A[1, 2] * B[2, 2] + A[1, 3] * B[3, 2] + A[1, 4] * B[4, 2] + A[1, 5] * B[5, 2];

            // --- RIGA 2 ---
            R[2, 0] = A[2, 0] * B[0, 0] + A[2, 1] * B[1, 0] + A[2, 2] * B[2, 0] + A[2, 3] * B[3, 0] + A[2, 4] * B[4, 0] + A[2, 5] * B[5, 0];
            R[2, 1] = A[2, 0] * B[0, 1] + A[2, 1] * B[1, 1] + A[2, 2] * B[2, 1] + A[2, 3] * B[3, 1] + A[2, 4] * B[4, 1] + A[2, 5] * B[5, 1];
            R[2, 2] = A[2, 0] * B[0, 2] + A[2, 1] * B[1, 2] + A[2, 2] * B[2, 2] + A[2, 3] * B[3, 2] + A[2, 4] * B[4, 2] + A[2, 5] * B[5, 2];
        }
        else
        {
            Debug.Log("Dimensions mismatch for 3x6 * 6x3 multiplication, returning...");
        }

        return R;
    }

    public static float[,] multiply63by36(float[,] A, float[,] B)
    {
        // (6x3) * (3x6) = 6x6
        float[,] R = Zeros66();
        if (A.GetLength(0) == 6 && A.GetLength(1) == 3 && B.GetLength(0) == 3 && B.GetLength(1) == 6)
        {
            R[0, 0] = A[0, 0] * B[0, 0] + A[0, 1] * B[1, 0] + A[0, 2] * B[2, 0];
            R[0, 1] = A[0, 0] * B[0, 1] + A[0, 1] * B[1, 1] + A[0, 2] * B[2, 1];
            R[0, 2] = A[0, 0] * B[0, 2] + A[0, 1] * B[1, 2] + A[0, 2] * B[2, 2];
            R[0, 3] = A[0, 0] * B[0, 3] + A[0, 1] * B[1, 3] + A[0, 2] * B[2, 3];
            R[0, 4] = A[0, 0] * B[0, 4] + A[0, 1] * B[1, 4] + A[0, 2] * B[2, 4];
            R[0, 5] = A[0, 0] * B[0, 5] + A[0, 1] * B[1, 5] + A[0, 2] * B[2, 5];

            R[1, 0] = A[1, 0] * B[0, 0] + A[1, 1] * B[1, 0] + A[1, 2] * B[2, 0];
            R[1, 1] = A[1, 0] * B[0, 1] + A[1, 1] * B[1, 1] + A[1, 2] * B[2, 1];
            R[1, 2] = A[1, 0] * B[0, 2] + A[1, 1] * B[1, 2] + A[1, 2] * B[2, 2];
            R[1, 3] = A[1, 0] * B[0, 3] + A[1, 1] * B[1, 3] + A[1, 2] * B[2, 3];
            R[1, 4] = A[1, 0] * B[0, 4] + A[1, 1] * B[1, 4] + A[1, 2] * B[2, 4];
            R[1, 5] = A[1, 0] * B[0, 5] + A[1, 1] * B[1, 5] + A[1, 2] * B[2, 5];

            R[2, 0] = A[2, 0] * B[0, 0] + A[2, 1] * B[1, 0] + A[2, 2] * B[2, 0];
            R[2, 1] = A[2, 0] * B[0, 1] + A[2, 1] * B[1, 1] + A[2, 2] * B[2, 1];
            R[2, 2] = A[2, 0] * B[0, 2] + A[2, 1] * B[1, 2] + A[2, 2] * B[2, 2];
            R[2, 3] = A[2, 0] * B[0, 3] + A[2, 1] * B[1, 3] + A[2, 2] * B[2, 3];
            R[2, 4] = A[2, 0] * B[0, 4] + A[2, 1] * B[1, 4] + A[2, 2] * B[2, 4];
            R[2, 5] = A[2, 0] * B[0, 5] + A[2, 1] * B[1, 5] + A[2, 2] * B[2, 5];

            R[3, 0] = A[3, 0] * B[0, 0] + A[3, 1] * B[1, 0] + A[3, 2] * B[2, 0];
            R[3, 1] = A[3, 0] * B[0, 1] + A[3, 1] * B[1, 1] + A[3, 2] * B[2, 1];
            R[3, 2] = A[3, 0] * B[0, 2] + A[3, 1] * B[1, 2] + A[3, 2] * B[2, 2];
            R[3, 3] = A[3, 0] * B[0, 3] + A[3, 1] * B[1, 3] + A[3, 2] * B[2, 3];
            R[3, 4] = A[3, 0] * B[0, 4] + A[3, 1] * B[1, 4] + A[3, 2] * B[2, 4];
            R[3, 5] = A[3, 0] * B[0, 5] + A[3, 1] * B[1, 5] + A[3, 2] * B[2, 5];

            R[4, 0] = A[4, 0] * B[0, 0] + A[4, 1] * B[1, 0] + A[4, 2] * B[2, 0];
            R[4, 1] = A[4, 0] * B[0, 1] + A[4, 1] * B[1, 1] + A[4, 2] * B[2, 1];
            R[4, 2] = A[4, 0] * B[0, 2] + A[4, 1] * B[1, 2] + A[4, 2] * B[2, 2];
            R[4, 3] = A[4, 0] * B[0, 3] + A[4, 1] * B[1, 3] + A[4, 2] * B[2, 3];
            R[4, 4] = A[4, 0] * B[0, 4] + A[4, 1] * B[1, 4] + A[4, 2] * B[2, 4];
            R[4, 5] = A[4, 0] * B[0, 5] + A[4, 1] * B[1, 5] + A[4, 2] * B[2, 5];

            R[5, 0] = A[5, 0] * B[0, 0] + A[5, 1] * B[1, 0] + A[5, 2] * B[2, 0];
            R[5, 1] = A[5, 0] * B[0, 1] + A[5, 1] * B[1, 1] + A[5, 2] * B[2, 1];
            R[5, 2] = A[5, 0] * B[0, 2] + A[5, 1] * B[1, 2] + A[5, 2] * B[2, 2];
            R[5, 3] = A[5, 0] * B[0, 3] + A[5, 1] * B[1, 3] + A[5, 2] * B[2, 3];
            R[5, 4] = A[5, 0] * B[0, 4] + A[5, 1] * B[1, 4] + A[5, 2] * B[2, 4];
            R[5, 5] = A[5, 0] * B[0, 5] + A[5, 1] * B[1, 5] + A[5, 2] * B[2, 5];
        }
        else
        {
            Debug.Log("Dimensions mismatch for 6x3 * 3x6 multiplication, returning...");
        }
        return R;
    }

    // Gauss-Jordan su matrice aumentata 6x7 [H | -b], senza pivoting.
    // Funziona correttamente se H è definita positiva (caso nominale ICP).
    public static float[] Solve6x6(float[,] H, float[] b)
    {
        float a00 = H[0,0]; float a01 = H[0,1]; float a02 = H[0,2]; float a03 = H[0,3]; float a04 = H[0,4]; float a05 = H[0,5]; float a06 = -b[0];
        float a10 = H[1,0]; float a11 = H[1,1]; float a12 = H[1,2]; float a13 = H[1,3]; float a14 = H[1,4]; float a15 = H[1,5]; float a16 = -b[1];
        float a20 = H[2,0]; float a21 = H[2,1]; float a22 = H[2,2]; float a23 = H[2,3]; float a24 = H[2,4]; float a25 = H[2,5]; float a26 = -b[2];
        float a30 = H[3,0]; float a31 = H[3,1]; float a32 = H[3,2]; float a33 = H[3,3]; float a34 = H[3,4]; float a35 = H[3,5]; float a36 = -b[3];
        float a40 = H[4,0]; float a41 = H[4,1]; float a42 = H[4,2]; float a43 = H[4,3]; float a44 = H[4,4]; float a45 = H[4,5]; float a46 = -b[4];
        float a50 = H[5,0]; float a51 = H[5,1]; float a52 = H[5,2]; float a53 = H[5,3]; float a54 = H[5,4]; float a55 = H[5,5]; float a56 = -b[5];

        // --- Colonna 0: normalizza riga 0, elimina dalle righe 1-5 ---
        if (Mathf.Abs(a00) < 1e-10f) { Debug.Log("Solve6x6: singolare alla colonna 0"); return Zeros6(); }
        float inv0 = 1f / a00;
        a00 *= inv0; a01 *= inv0; a02 *= inv0; a03 *= inv0; a04 *= inv0; a05 *= inv0; a06 *= inv0;
        float f10 = a10; a10 -= f10*a00; a11 -= f10*a01; a12 -= f10*a02; a13 -= f10*a03; a14 -= f10*a04; a15 -= f10*a05; a16 -= f10*a06;
        float f20 = a20; a20 -= f20*a00; a21 -= f20*a01; a22 -= f20*a02; a23 -= f20*a03; a24 -= f20*a04; a25 -= f20*a05; a26 -= f20*a06;
        float f30 = a30; a30 -= f30*a00; a31 -= f30*a01; a32 -= f30*a02; a33 -= f30*a03; a34 -= f30*a04; a35 -= f30*a05; a36 -= f30*a06;
        float f40 = a40; a40 -= f40*a00; a41 -= f40*a01; a42 -= f40*a02; a43 -= f40*a03; a44 -= f40*a04; a45 -= f40*a05; a46 -= f40*a06;
        float f50 = a50; a50 -= f50*a00; a51 -= f50*a01; a52 -= f50*a02; a53 -= f50*a03; a54 -= f50*a04; a55 -= f50*a05; a56 -= f50*a06;

        // --- Colonna 1: normalizza riga 1, elimina dalle righe 0,2-5 ---
        if (Mathf.Abs(a11) < 1e-10f) { Debug.Log("Solve6x6: singolare alla colonna 1"); return Zeros6(); }
        float inv1 = 1f / a11;
        a10 *= inv1; a11 *= inv1; a12 *= inv1; a13 *= inv1; a14 *= inv1; a15 *= inv1; a16 *= inv1;
        float f01 = a01; a00 -= f01*a10; a01 -= f01*a11; a02 -= f01*a12; a03 -= f01*a13; a04 -= f01*a14; a05 -= f01*a15; a06 -= f01*a16;
        float f21 = a21; a20 -= f21*a10; a21 -= f21*a11; a22 -= f21*a12; a23 -= f21*a13; a24 -= f21*a14; a25 -= f21*a15; a26 -= f21*a16;
        float f31 = a31; a30 -= f31*a10; a31 -= f31*a11; a32 -= f31*a12; a33 -= f31*a13; a34 -= f31*a14; a35 -= f31*a15; a36 -= f31*a16;
        float f41 = a41; a40 -= f41*a10; a41 -= f41*a11; a42 -= f41*a12; a43 -= f41*a13; a44 -= f41*a14; a45 -= f41*a15; a46 -= f41*a16;
        float f51 = a51; a50 -= f51*a10; a51 -= f51*a11; a52 -= f51*a12; a53 -= f51*a13; a54 -= f51*a14; a55 -= f51*a15; a56 -= f51*a16;

        // --- Colonna 2: normalizza riga 2, elimina dalle righe 0,1,3-5 ---
        if (Mathf.Abs(a22) < 1e-10f) { Debug.Log("Solve6x6: singolare alla colonna 2"); return Zeros6(); }
        float inv2 = 1f / a22;
        a20 *= inv2; a21 *= inv2; a22 *= inv2; a23 *= inv2; a24 *= inv2; a25 *= inv2; a26 *= inv2;
        float f02 = a02; a00 -= f02*a20; a01 -= f02*a21; a02 -= f02*a22; a03 -= f02*a23; a04 -= f02*a24; a05 -= f02*a25; a06 -= f02*a26;
        float f12 = a12; a10 -= f12*a20; a11 -= f12*a21; a12 -= f12*a22; a13 -= f12*a23; a14 -= f12*a24; a15 -= f12*a25; a16 -= f12*a26;
        float f32 = a32; a30 -= f32*a20; a31 -= f32*a21; a32 -= f32*a22; a33 -= f32*a23; a34 -= f32*a24; a35 -= f32*a25; a36 -= f32*a26;
        float f42 = a42; a40 -= f42*a20; a41 -= f42*a21; a42 -= f42*a22; a43 -= f42*a23; a44 -= f42*a24; a45 -= f42*a25; a46 -= f42*a26;
        float f52 = a52; a50 -= f52*a20; a51 -= f52*a21; a52 -= f52*a22; a53 -= f52*a23; a54 -= f52*a24; a55 -= f52*a25; a56 -= f52*a26;

        // --- Colonna 3: normalizza riga 3, elimina dalle righe 0-2,4,5 ---
        if (Mathf.Abs(a33) < 1e-10f) { Debug.Log("Solve6x6: singolare alla colonna 3"); return Zeros6(); }
        float inv3 = 1f / a33;
        a30 *= inv3; a31 *= inv3; a32 *= inv3; a33 *= inv3; a34 *= inv3; a35 *= inv3; a36 *= inv3;
        float f03 = a03; a00 -= f03*a30; a01 -= f03*a31; a02 -= f03*a32; a03 -= f03*a33; a04 -= f03*a34; a05 -= f03*a35; a06 -= f03*a36;
        float f13 = a13; a10 -= f13*a30; a11 -= f13*a31; a12 -= f13*a32; a13 -= f13*a33; a14 -= f13*a34; a15 -= f13*a35; a16 -= f13*a36;
        float f23 = a23; a20 -= f23*a30; a21 -= f23*a31; a22 -= f23*a32; a23 -= f23*a33; a24 -= f23*a34; a25 -= f23*a35; a26 -= f23*a36;
        float f43 = a43; a40 -= f43*a30; a41 -= f43*a31; a42 -= f43*a32; a43 -= f43*a33; a44 -= f43*a34; a45 -= f43*a35; a46 -= f43*a36;
        float f53 = a53; a50 -= f53*a30; a51 -= f53*a31; a52 -= f53*a32; a53 -= f53*a33; a54 -= f53*a34; a55 -= f53*a35; a56 -= f53*a36;

        // --- Colonna 4: normalizza riga 4, elimina dalle righe 0-3,5 ---
        if (Mathf.Abs(a44) < 1e-10f) { Debug.Log("Solve6x6: singolare alla colonna 4"); return Zeros6(); }
        float inv4 = 1f / a44;
        a40 *= inv4; a41 *= inv4; a42 *= inv4; a43 *= inv4; a44 *= inv4; a45 *= inv4; a46 *= inv4;
        float f04 = a04; a00 -= f04*a40; a01 -= f04*a41; a02 -= f04*a42; a03 -= f04*a43; a04 -= f04*a44; a05 -= f04*a45; a06 -= f04*a46;
        float f14 = a14; a10 -= f14*a40; a11 -= f14*a41; a12 -= f14*a42; a13 -= f14*a43; a14 -= f14*a44; a15 -= f14*a45; a16 -= f14*a46;
        float f24 = a24; a20 -= f24*a40; a21 -= f24*a41; a22 -= f24*a42; a23 -= f24*a43; a24 -= f24*a44; a25 -= f24*a45; a26 -= f24*a46;
        float f34 = a34; a30 -= f34*a40; a31 -= f34*a41; a32 -= f34*a42; a33 -= f34*a43; a34 -= f34*a44; a35 -= f34*a45; a36 -= f34*a46;
        float f54 = a54; a50 -= f54*a40; a51 -= f54*a41; a52 -= f54*a42; a53 -= f54*a43; a54 -= f54*a44; a55 -= f54*a45; a56 -= f54*a46;

        // --- Colonna 5: normalizza riga 5, elimina dalle righe 0-4 ---
        if (Mathf.Abs(a55) < 1e-10f) { Debug.Log("Solve6x6: singolare alla colonna 5"); return Zeros6(); }
        float inv5 = 1f / a55;
        a50 *= inv5; a51 *= inv5; a52 *= inv5; a53 *= inv5; a54 *= inv5; a55 *= inv5; a56 *= inv5;
        float f05 = a05; a00 -= f05*a50; a01 -= f05*a51; a02 -= f05*a52; a03 -= f05*a53; a04 -= f05*a54; a05 -= f05*a55; a06 -= f05*a56;
        float f15 = a15; a10 -= f15*a50; a11 -= f15*a51; a12 -= f15*a52; a13 -= f15*a53; a14 -= f15*a54; a15 -= f15*a55; a16 -= f15*a56;
        float f25 = a25; a20 -= f25*a50; a21 -= f25*a51; a22 -= f25*a52; a23 -= f25*a53; a24 -= f25*a54; a25 -= f25*a55; a26 -= f25*a56;
        float f35 = a35; a30 -= f35*a50; a31 -= f35*a51; a32 -= f35*a52; a33 -= f35*a53; a34 -= f35*a54; a35 -= f35*a55; a36 -= f35*a56;
        float f45 = a45; a40 -= f45*a50; a41 -= f45*a51; a42 -= f45*a52; a43 -= f45*a53; a44 -= f45*a54; a45 -= f45*a55; a46 -= f45*a56;

        return new float[] { a06, a16, a26, a36, a46, a56 };
    }


    public static float[,] Assign6x6AtIndex(float[,] startingM, float[,] mToBeAssigned, int rs, int cs)        
    {
        float[,] returnM = startingM;
        if(mToBeAssigned.GetLength(0)==6 && mToBeAssigned.GetLength(1)==6 && startingM.GetLength(0)>=6 && startingM.GetLength(1) >= 6)
        {

            int rs_r = rs;
            int cs_r = cs;

            returnM[rs_r, cs_r] = mToBeAssigned[0, 0];
            returnM[rs_r, cs_r+1] = mToBeAssigned[0, 1];
            returnM[rs_r, cs_r+2] = mToBeAssigned[0, 2];
            returnM[rs_r, cs_r+3] = mToBeAssigned[0, 3];
            returnM[rs_r, cs_r+4] = mToBeAssigned[0, 4];
            returnM[rs_r, cs_r+5] = mToBeAssigned[0, 5];

            returnM[rs_r+1, cs_r] = mToBeAssigned[1, 0];
            returnM[rs_r+1, cs_r + 1] = mToBeAssigned[1, 1];
            returnM[rs_r+1, cs_r + 2] = mToBeAssigned[1, 2];
            returnM[rs_r+1, cs_r + 3] = mToBeAssigned[1, 3];
            returnM[rs_r+1, cs_r + 4] = mToBeAssigned[1, 4];
            returnM[rs_r+1, cs_r + 5] = mToBeAssigned[1, 5];

            returnM[rs_r + 2, cs_r] = mToBeAssigned[2, 0];
            returnM[rs_r + 2, cs_r + 1] = mToBeAssigned[2, 1];
            returnM[rs_r + 2, cs_r + 2] = mToBeAssigned[2, 2];
            returnM[rs_r + 2, cs_r + 3] = mToBeAssigned[2, 3];
            returnM[rs_r + 2, cs_r + 4] = mToBeAssigned[2, 4];
            returnM[rs_r + 2, cs_r + 5] = mToBeAssigned[2, 5];

            returnM[rs_r + 3, cs_r] = mToBeAssigned[3, 0];
            returnM[rs_r + 3, cs_r + 1] = mToBeAssigned[3, 1];
            returnM[rs_r + 3, cs_r + 2] = mToBeAssigned[3, 2];
            returnM[rs_r + 3, cs_r + 3] = mToBeAssigned[3, 3];
            returnM[rs_r + 3, cs_r + 4] = mToBeAssigned[3, 4];
            returnM[rs_r + 3, cs_r + 5] = mToBeAssigned[3, 5];

            returnM[rs_r + 4, cs_r] = mToBeAssigned[4, 0];
            returnM[rs_r + 4, cs_r + 1] = mToBeAssigned[4, 1];
            returnM[rs_r + 4, cs_r + 2] = mToBeAssigned[4, 2];
            returnM[rs_r + 4, cs_r + 3] = mToBeAssigned[4, 3];
            returnM[rs_r + 4, cs_r + 4] = mToBeAssigned[4, 4];
            returnM[rs_r + 4, cs_r + 5] = mToBeAssigned[4, 5];

            returnM[rs_r + 5, cs_r] = mToBeAssigned[5, 0];
            returnM[rs_r + 5, cs_r + 1] = mToBeAssigned[5, 1];
            returnM[rs_r + 5, cs_r + 2] = mToBeAssigned[5, 2];
            returnM[rs_r + 5, cs_r + 3] = mToBeAssigned[5, 3];
            returnM[rs_r + 5, cs_r + 4] = mToBeAssigned[5, 4];
            returnM[rs_r + 5, cs_r + 5] = mToBeAssigned[5, 5];
        }
        else
        {
            Debug.Log("Dimension error in assing 6x6 matrix function, returning...");
        }
        return returnM;
    }


    public static float[,] AssignAccumulating6x6AtIndex(float[,] startingM, float[,] mToBeAssigned, int rs, int cs)
    {
        float[,] returnM = startingM;
        if (mToBeAssigned.GetLength(0) == 6 && mToBeAssigned.GetLength(1) == 6 && startingM.GetLength(0) >= 6 && startingM.GetLength(1) >= 6)
        {

            int rs_r = rs;
            int cs_r = cs;

            returnM[rs_r, cs_r] += mToBeAssigned[0, 0];
            returnM[rs_r, cs_r + 1] += mToBeAssigned[0, 1];
            returnM[rs_r, cs_r + 2] += mToBeAssigned[0, 2];
            returnM[rs_r, cs_r + 3] += mToBeAssigned[0, 3];
            returnM[rs_r, cs_r + 4] += mToBeAssigned[0, 4];
            returnM[rs_r, cs_r + 5] += mToBeAssigned[0, 5];

            returnM[rs_r + 1, cs_r] += mToBeAssigned[1, 0];
            returnM[rs_r + 1, cs_r + 1] += mToBeAssigned[1, 1];
            returnM[rs_r + 1, cs_r + 2] += mToBeAssigned[1, 2];
            returnM[rs_r + 1, cs_r + 3] += mToBeAssigned[1, 3];
            returnM[rs_r + 1, cs_r + 4] += mToBeAssigned[1, 4];
            returnM[rs_r + 1, cs_r + 5] += mToBeAssigned[1, 5];

            returnM[rs_r + 2, cs_r] += mToBeAssigned[2, 0];
            returnM[rs_r + 2, cs_r + 1] += mToBeAssigned[2, 1];
            returnM[rs_r + 2, cs_r + 2] += mToBeAssigned[2, 2];
            returnM[rs_r + 2, cs_r + 3] += mToBeAssigned[2, 3];
            returnM[rs_r + 2, cs_r + 4] += mToBeAssigned[2, 4];
            returnM[rs_r + 2, cs_r + 5] += mToBeAssigned[2, 5];

            returnM[rs_r + 3, cs_r] += mToBeAssigned[3, 0];
            returnM[rs_r + 3, cs_r + 1] += mToBeAssigned[3, 1];
            returnM[rs_r + 3, cs_r + 2] += mToBeAssigned[3, 2];
            returnM[rs_r + 3, cs_r + 3] += mToBeAssigned[3, 3];
            returnM[rs_r + 3, cs_r + 4] += mToBeAssigned[3, 4];
            returnM[rs_r + 3, cs_r + 5] += mToBeAssigned[3, 5];

            returnM[rs_r + 4, cs_r] += mToBeAssigned[4, 0];
            returnM[rs_r + 4, cs_r + 1] += mToBeAssigned[4, 1];
            returnM[rs_r + 4, cs_r + 2] += mToBeAssigned[4, 2];
            returnM[rs_r + 4, cs_r + 3] += mToBeAssigned[4, 3];
            returnM[rs_r + 4, cs_r + 4] += mToBeAssigned[4, 4];
            returnM[rs_r + 4, cs_r + 5] += mToBeAssigned[4, 5];

            returnM[rs_r + 5, cs_r] += mToBeAssigned[5, 0];
            returnM[rs_r + 5, cs_r + 1] += mToBeAssigned[5, 1];
            returnM[rs_r + 5, cs_r + 2] += mToBeAssigned[5, 2];
            returnM[rs_r + 5, cs_r + 3] += mToBeAssigned[5, 3];
            returnM[rs_r + 5, cs_r + 4] += mToBeAssigned[5, 4];
            returnM[rs_r + 5, cs_r + 5] += mToBeAssigned[5, 5];
        }
        else
        {
            Debug.Log("Dimension error in assining 6x6 matrix function, returning...");
        }
        return returnM;
    }

    public static float[] AssignAccumulating6AtIndex(float[] startingV, float[] vToBeAssigned, int index)
    {
        float[] returnV = startingV;
        if (vToBeAssigned.GetLength(0) == 6 && startingV.GetLength(0) >= 6)
        {

            int index_s = index;

            returnV[index_s] += vToBeAssigned[0];
            returnV[index_s + 1] += vToBeAssigned[1];
            returnV[index_s + 2] += vToBeAssigned[2];
            returnV[index_s + 3] += vToBeAssigned[3];
            returnV[index_s + 4] += vToBeAssigned[4];
            returnV[index_s + 5] += vToBeAssigned[5];
        }
        else
        {
            Debug.Log("Dimension error in assining 6 dimensional vector, returning...");
        }
        return returnV;
    }

}

