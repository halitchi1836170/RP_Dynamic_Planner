using System;
using UnityEngine;

public static class TridiagonalSolver
{
    
    public static double[] SolveTridiagonalMatrix(double[] a, double[] b, double[] c, double[] d)
    {
        int n = b.Length;
        if (a.Length != n || c.Length != n || d.Length != n)
            throw new ArgumentException("Tutti i vettori devono avere la stessa lunghezza.");

        double[] cp = new double[n];   // c'  (coefficienti modificati)
        double[] dp = new double[n];   // d'
        double[] x = new double[n];

        // --- forward sweep ---
        double pivot = b[0];
        if (Math.Abs(pivot) < 1e-12)
            throw new InvalidOperationException("Pivot nullo alla riga 0: matrice singolare o mal posta.");

        cp[0] = c[0] / pivot;
        dp[0] = d[0] / pivot;

        for (int i = 1; i < n; i++)
        {
            pivot = b[i] - a[i] * cp[i - 1];
            if (Math.Abs(pivot) < 1e-12)
                throw new InvalidOperationException($"Pivot nullo alla riga {i}: perdita di dominanza diagonale?");

            cp[i] = c[i] / pivot;
            dp[i] = (d[i] - a[i] * dp[i - 1]) / pivot;
        }

        // --- back substitution ---
        x[n - 1] = dp[n - 1];
        for (int i = n - 2; i >= 0; i--)
            x[i] = dp[i] - cp[i] * x[i + 1];

        return x;
    }
}
