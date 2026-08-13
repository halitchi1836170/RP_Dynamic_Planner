using UnityEngine;
using static MatrixVectorUtilities;
using static PoseMatrix4x4;

// Localizzazione scan-to-map (predict -> correct -> gate).
//  - PREDICT: guess = posa precedente composta col moto relativo dell'odometria di ruota (dead-reckoning),
//    cosi' il guess SEGUE il robot tra le correzioni e nei rifiuti -> l'ICP parte sempre vicino alla verita'.
//  - CORRECT: ICP della scansione live contro la mappa salvata (icpService.LocalizeAgainstMap).
//  - GATE: accetto solo se affidabile (inlierRatio/residual) E il "salto" corrected-vs-guess e' plausibile
//    (maxJump). Se rifiuto uso il guess (dead-reckoning odometrico), NON la posa vecchia: cosi' la stima
//    avanza comunque col robot e il guess non invecchia.
// Stato T_map_laser: posa del laser in world (Unity, 4x4). Ritorno la posa in frame ROS (x,y,theta).
public class LocalizationService
{
    private ICPService icpService;
    private Transform laserTransform;
    private float minInlierRatio;
    private float maxResidual;
    private float maxJump;                       // salto max [m] corrected-vs-guess: oltre -> convergenza errata

    private float[,] T_map_laser;
    private bool initialized;

    private (float x, float y, float theta) prevOdo;   // odometria all'ultimo Step (per il delta relativo)
    private bool hasPrevOdo;

    public LocalizationService(ICPService icpService, Transform laserTransform, float minInlierRatio, float maxResidual, float maxJump)
    {
        this.icpService = icpService;
        this.laserTransform = laserTransform;
        this.minInlierRatio = minInlierRatio;
        this.maxResidual = maxResidual;
        this.maxJump = maxJump;
        this.T_map_laser = Identity4();
        this.initialized = false;
        this.hasPrevOdo = false;
    }

    // Posa iniziale nota = posa del laser (transform live) come 4x4 local->world (colonne = right/up/forward).
    public void Initialize()
    {
        Vector3 p = laserTransform.position, r = laserTransform.right, u = laserTransform.up, f = laserTransform.forward;
        float[,] R = composeRByColumnVectors(new float[] { r.x, r.y, r.z }, new float[] { u.x, u.y, u.z }, new float[] { f.x, f.y, f.z });
        T_map_laser = composeT(R, new float[] { p.x, p.y, p.z });
        initialized = true;
    }

    // Un passo predict->correct->gate. 'currentOdo' = posa corrente dell'odometria di ruota (frame O: x,y,theta).
    public (float x, float y, float theta, bool accepted) Step((float x, float y, float theta) currentOdo)
    {
        if (!initialized) Initialize();

        // PREDICT: guess = T_map_laser * ΔT_odom (moto relativo dall'ultimo Step). Al primo giro ΔT = Identita'.
        float[,] guess = T_map_laser;
        if (hasPrevOdo)
        {
            float[,] deltaOdo = productSquareMatrix4(InverseT(odoToWorldT(prevOdo)), odoToWorldT(currentOdo));
            guess = productSquareMatrix4(T_map_laser, deltaOdo);
        }
        prevOdo = currentOdo;
        hasPrevOdo = true;

        // CORRECT
        (float[,] correctedPose, float residual, float inlierRatio) icp = icpService.LocalizeAgainstMap(guess);

        // GATE: fitness + salto plausibile (una convergenza sbagliata sposta la posa di molto rispetto al guess).
        float jump = translationDistance(icp.correctedPose, guess);
        bool accepted = icp.inlierRatio >= minInlierRatio && icp.residual <= maxResidual && jump <= maxJump;

        // Accettato -> ICP; rifiutato -> guess (dead-reckoning odometrico), non la posa vecchia.
        T_map_laser = accepted ? icp.correctedPose : guess;

        (float rx, float ry, float rth) = toRosPose(T_map_laser);
        return (rx, ry, rth, accepted);
    }

    // Ricostruisce la posa del laser in Unity-world dall'odometria (x_o=z, y_o=x, theta_o=yaw): e' l'inversa di
    // getTransformCurrentConfiguration. Y arbitrario: si annulla nel delta relativo (prev e curr hanno lo stesso Y).
    private float[,] odoToWorldT((float x, float y, float theta) odo)
    {
        Quaternion rot = Quaternion.Euler(0f, odo.theta * Mathf.Rad2Deg, 0f);
        Vector3 r = rot * Vector3.right, u = rot * Vector3.up, f = rot * Vector3.forward;
        float[,] R = composeRByColumnVectors(new float[] { r.x, r.y, r.z }, new float[] { u.x, u.y, u.z }, new float[] { f.x, f.y, f.z });
        return composeT(R, new float[] { odo.y, 0f, odo.x });   // pos.x = y_o, pos.z = x_o
    }

    private float translationDistance(float[,] A, float[,] B)
    {
        float dx = A[0, 3] - B[0, 3], dy = A[1, 3] - B[1, 3], dz = A[2, 3] - B[2, 3];
        return Mathf.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    // 4x4 Unity-world -> (x,y,theta) ROS. Posizione: UnityToRos = (z, -x). Heading: forward (3a colonna) -> ROS.
    private (float x, float y, float theta) toRosPose(float[,] T)
    {
        float x = T[2, 3];
        float y = -T[0, 3];
        float theta = Mathf.Atan2(-T[0, 2], T[2, 2]);
        return (x, y, theta);
    }

    public (float x, float y, float theta) GetRosPose() => toRosPose(T_map_laser);
}
