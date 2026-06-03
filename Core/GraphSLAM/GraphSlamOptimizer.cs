using System;
using System.Collections.Generic;
using UnityEngine;
using static MatrixVectorUtilities;
using static PoseMatrix4x4;

public class GraphSlamOptimizer
{

    private int maxIterations;
    private int maxCGIterations;
    private float convergenceThreshold;
    private float convergenceCGThreshold;

    private PoseGraph poseGraph;

    public GraphSlamOptimizer(int MaxIterations, int MaxCGIterations, float convergenceGraphSlamOptimizerThreshold, float convergenceCGAlgorithm, PoseGraph PoseGraph)
    {
        this.maxIterations = MaxIterations;
        this.maxCGIterations = MaxCGIterations;
        this.poseGraph = PoseGraph;
        this.convergenceThreshold = convergenceGraphSlamOptimizerThreshold;
        this.convergenceCGThreshold = convergenceCGAlgorithm;
    }

    private class GraphMatrixBlock6x6{
        public int RowBlockIndex;    // Indice del nodo riga
        public int ColBlockIndex;    // Indice del nodo colonna
        public float[,] Values = new float[6,6]; // I 36 elementi del blocco 6x6 (flat array per performance)

        public GraphMatrixBlock6x6(int r, int c)
        {
            this.RowBlockIndex = r;
            this.ColBlockIndex = c;
        }

        public void ClearZero()
        {
            System.Array.Clear(Values, 0, 36);
        }

        // Usa questo invece di riassegnare l'intero array
        public void AddValues(float[,] newValues)
        {
            for (int r = 0; r < 6; r++)
                for (int c = 0; c < 6; c++)
                    Values[r, c] += newValues[r, c];
        }
    }

    public void Optimize()
    {
        int NNodes = poseGraph.getNNodes();
        int totalSize = NNodes * 6;

        List<GraphMatrixBlock6x6> hBlocksList = new List<GraphMatrixBlock6x6>();                        //questa sarà comoda per la moltiplicazione sparsa
        Dictionary<int, GraphMatrixBlock6x6> hBlocksMap = new Dictionary<int, GraphMatrixBlock6x6>();   //questa è comoda per l'indicizzazione delle coppie di indici/blocchi

        foreach (PoseEdge edge in poseGraph.Edges())
        {
            int i = edge.FromNodeID() - 1;
            int j = edge.ToNodeID() - 1;
            RegisterBlockIfNeeded(i, i, NNodes, hBlocksList, hBlocksMap);
            RegisterBlockIfNeeded(j, j, NNodes, hBlocksList, hBlocksMap);
            RegisterBlockIfNeeded(i, j, NNodes, hBlocksList, hBlocksMap);
            RegisterBlockIfNeeded(j, i, NNodes, hBlocksList, hBlocksMap);
        }
        // Registriamo il blocco (0,0) per l'Identity (fissare la gauge del grafo)
        RegisterBlockIfNeeded(0, 0, NNodes, hBlocksList, hBlocksMap);

        float[] b = new float[totalSize];
        float[] dx = new float[totalSize]; // per il risultato del CG

        for (int iter=0; iter<maxIterations; iter++)
        {
            System.Array.Clear(b, 0, totalSize);
            foreach (var block in hBlocksList)
            {
                block.ClearZero();
            }

            foreach (PoseEdge edge in poseGraph.Edges())
            {
                int i = edge.FromNodeID() - 1;
                int j = edge.ToNodeID() - 1;

                int edge_fromID = edge.FromNodeID();
                int edge_toID = edge.ToNodeID();

                float[,] T_i = poseGraph.getNode(edge_fromID).PoseT();
                float[,] T_j = poseGraph.getNode(edge_toID).PoseT();
                float[,] z_ij = edge.RelativeT();
                
                float[,] Omega_ij = edge.InformationM();

                float[] e_ij = LogMap(productSquareMatrix4(productSquareMatrix4(InverseT(z_ij), InverseT(T_i)), T_j));

                // Jacobiani analitici SE(3) (perturbazione destra X <- X * Exp(delta)):
                //   J_j =  J_r^-1(e)               ~= I + 1/2 ad_e
                //   J_i = -J_l^-1(e) * Adj(Z^-1)   ~= -(I - 1/2 ad_e) * Adj(InverseT(z))
                // L'adjoint e' esatto (termine dominante che accoppia rotazione e traslazione);
                // J_r^-1 / J_l^-1 sono troncati al primo ordine della serie (-> I quando e -> 0).
                float[,] adE = littleAdjoint6(e_ij);
                float[,] Jr_inv = sumSquareMatrix6(Identity6(), productSquareMatrix6Scalar(adE, 0.5f));
                float[,] Jl_inv = sumSquareMatrix6(Identity6(), productSquareMatrix6Scalar(adE, -0.5f));
                float[,] AdjZinv = AdjointSE3(InverseT(z_ij));

                float[,] J_j = Jr_inv;
                float[,] J_i = productSquareMatrix6Scalar(productSquareMatrix6(Jl_inv, AdjZinv), -1.0f);

                float[,] Ji_T = transposeSquareMatrix6(J_i);
                float[,] Jj_T = transposeSquareMatrix6(J_j);

                // Pre-moltiplicazioni J^T * Omega (con Omega = I risultano J^T, ma teniamo il caso generale)
                float[,] JiT_O = productSquareMatrix6(Ji_T, Omega_ij);
                float[,] JjT_O = productSquareMatrix6(Jj_T, Omega_ij);

                // Blocchi di H = J^T Omega J
                hBlocksMap[GetBlockID(i, i, NNodes)].AddValues(productSquareMatrix6(JiT_O, J_i));
                hBlocksMap[GetBlockID(j, j, NNodes)].AddValues(productSquareMatrix6(JjT_O, J_j));
                hBlocksMap[GetBlockID(i, j, NNodes)].AddValues(productSquareMatrix6(JiT_O, J_j));
                hBlocksMap[GetBlockID(j, i, NNodes)].AddValues(productSquareMatrix6(JjT_O, J_i));

                // Gradiente b = J^T Omega e (verra' negato dopo il loop per risolvere H dx = -b)
                b = AssignAccumulating6AtIndex(b, productSquareMatrix6Vector6(JiT_O, e_ij), i * 6);
                b = AssignAccumulating6AtIndex(b, productSquareMatrix6Vector6(JjT_O, e_ij), j * 6);
            }

            for (int i = 0; i < totalSize; i++)
            {
                b[i] = -b[i];
            }

            foreach (var block in hBlocksList)
            {
                if (block.RowBlockIndex == 0 && block.ColBlockIndex == 0)
                {
                    // Sovrascrive completamente con I
                    block.ClearZero();
                    for (int k = 0; k < 6; k++) block.Values[k, k] = 1.0f;
                }
                else if (block.RowBlockIndex == 0 || block.ColBlockIndex == 0)
                {
                    // Azzera le interazioni fuori diagonale (H[0,j] e H[i,0])
                    block.ClearZero();
                }
            }

            // Azzeramento del vettore b per i 6 DOF del nodo 0
            System.Array.Clear(b, 0, 6);

            // Damping leggero di Levenberg-Marquardt sui blocchi diagonali: mantiene H
            // strettamente definita positiva per il CG senza distorcere la soluzione.
            float dampingLambda = 1e-4f;
            foreach (var block in hBlocksList)
            {
                if (block.RowBlockIndex == block.ColBlockIndex)
                    for (int k = 0; k < 6; k++) block.Values[k, k] += dampingLambda;
            }

            dx = SolveGenericLinearSystemUsingCG(hBlocksList, b, totalSize, maxCGIterations, convergenceCGThreshold);

            float dxNorm = getNorm(dx);
            Debug.Log($"GraphSLAM optimizer iter {iter}: dx norm = {dxNorm:F6}");
            if (dxNorm < convergenceThreshold) break;

            foreach (PoseNode node in poseGraph.Nodes())
            {
                if (node.PoseID() - 1 != 0)
                {
                    float[] subDelta = getSub6Vector(dx, (node.PoseID() - 1) * 6);
                    node.setPoseT(productSquareMatrix4(node.PoseT(), ExpMap(subDelta)));
                }
            }
        }

    }

    private float[] SolveGenericLinearSystemUsingCG(List<GraphMatrixBlock6x6> hBlocksList, float[] b, int totalSize, int maxCgIterations, float tolerance)
    {
        // Allocazione vettori (da fare idealmente fuori dal CG se chiamato ad ogni iterazione SLAM)
        float[] x = new float[totalSize]; // L'ipotesi iniziale x_0 è il vettore nullo
        float[] r = new float[totalSize];
        float[] p = new float[totalSize];
        float[] q = new float[totalSize];

        // Inizializzazione
        // Dato che x_0 = 0, il residuo iniziale r_0 = b - H*x_0 è semplicemente b.
        // E la prima direzione di ricerca p_0 è r_0.
        System.Array.Copy(b, r, totalSize);
        System.Array.Copy(b, p, totalSize);

        // Calcolo del (r^T * r) iniziale
        float rTr = 0;
        for (int i = 0; i < totalSize; i++)
        {
            rTr += r[i] * r[i];
        }

        for (int k = 0; k < maxCgIterations; k++)
        {
            // 1. Calcola q = H * p (Il cuore computazionale!)
            MultiplySparseBlockMatrix(hBlocksList, p, q, totalSize);

            // 2. Calcola p^T * q
            float pTq = 0;
            for (int i = 0; i < totalSize; i++)
            {
                pTq += p[i] * q[i];
            }

            // Sicurezza: previene divisioni per zero se H è mal condizionata
            if (System.Math.Abs(pTq) < 1e-12f) break;

            // 3. Calcola lo step size alpha
            float alpha = rTr / pTq;

            float rTr_next = 0;

            // 4. Aggiorna la soluzione x e il residuo r
            // Uniamo i due cicli per sfruttare la cache locality
            for (int i = 0; i < totalSize; i++)
            {
                x[i] += alpha * p[i];
                r[i] -= alpha * q[i];
                rTr_next += r[i] * r[i]; // Prepariamo già il prossimo rTr
            }

            // 5. Controllo di convergenza (Norma L2 del residuo)
            if (System.Math.Sqrt(rTr_next) < tolerance)
            {
                // Debug.Log($"CG Converged in {k} iterations.");
                break;
            }

            // 6. Calcola il fattore di ortogonalizzazione beta
            float beta = rTr_next / rTr;

            // 7. Aggiorna la direzione di ricerca p
            for (int i = 0; i < totalSize; i++)
            {
                p[i] = r[i] + beta * p[i];
            }

            // Prepara rTr per la prossima iterazione
            rTr = rTr_next;
        }

        return x; // Ritorna il nostro delta_x ottimizzato
    }

    private static void MultiplySparseBlockMatrix(List<GraphMatrixBlock6x6> blocks, float[] p, float[] q, int totalSize)
    {
        // Azzera il vettore risultato prima del calcolo
        System.Array.Clear(q, 0, totalSize);

        // Cicliamo SOLO sui blocchi effettivamente esistenti (niente zeri!)
        foreach (var block in blocks)
        {
            int rOffset = block.RowBlockIndex * 6;
            int cOffset = block.ColBlockIndex * 6;

            // Moltiplicazione hardcoded del blocco 6x6 per il sotto-vettore di p
            // Questo sfrutta appieno la cache della CPU
            for (int r = 0; r < 6; r++)
            {
                int rowIdx = rOffset + r;

                float sum = 0;
                sum += block.Values[r, 0] * p[cOffset + 0];
                sum += block.Values[r, 1] * p[cOffset + 1];
                sum += block.Values[r, 2] * p[cOffset + 2];
                sum += block.Values[r, 3] * p[cOffset + 3];
                sum += block.Values[r, 4] * p[cOffset + 4];
                sum += block.Values[r, 5] * p[cOffset + 5];

                q[rowIdx] += sum;
            }
        }
    }

    private void RegisterBlockIfNeeded(int r, int c, int NNodes, List<GraphMatrixBlock6x6> hBlocksList, Dictionary<int, GraphMatrixBlock6x6> hBlocksMap)
    {
        int id = GetBlockID(r, c, NNodes);
        if (!hBlocksMap.ContainsKey(id))
        {
            var newBlock = new GraphMatrixBlock6x6(r, c);
            hBlocksMap[id] = newBlock;
            hBlocksList.Add(newBlock); // Aggiungiamo anche alla lista per il CG
        }
    }

    private int GetBlockID(int r, int c, int totalNodes)
    {
        return r * totalNodes + c;
    }

}
