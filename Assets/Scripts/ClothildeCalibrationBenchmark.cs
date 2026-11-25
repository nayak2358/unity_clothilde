using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

/// <summary>
/// Scripted, non-VR calibration experiments for Clothilde.
///
/// Add this component to an empty GameObject and assign the TriangleMesh.
/// The script:
///   * applies deterministic boundary conditions,
///   * records all physical nodes at every simulated frame,
///   * stores material coordinates (u,v) for simulator-independent matching,
///   * writes one long-format CSV file per run.
///
/// IMPORTANT:
/// This component uses a negative execution order so Awake() runs before
/// TriangleMesh.Awake(). This lets it set the physical parameters before the
/// Python Cloth object and its matrices are constructed.
/// </summary>
[DefaultExecutionOrder(-1000)]
public class ClothildeCalibrationBenchmark : MonoBehaviour
{
    public enum CalibrationMode
    {
        StretchPulse,
        ShearPulse,
        CantileverSag,
        CantileverRelease
    }

    [Header("References")]
    public TriangleMesh cloth;

    [Header("Experiment")]
    public CalibrationMode mode = CalibrationMode.StretchPulse;
    public string runName = "clothilde_calibration";
    public bool applyRecommendedIsolationSettings = true;
    public bool stopPlayModeWhenFinished = true;

    [Header("Stretch / shear pulse")]
    [Tooltip("Initial period with both opposite edges held at their reference positions.")]
    public float pulseRestSeconds = 1.0f;
    public float pulseMoveSeconds = 0.5f;
    public float pulseHoldSeconds = 0.5f;
    public float pulseFreeSeconds = 3.0f;
    public float stretchAmplitude = 0.035f;
    public float shearAmplitude = 0.070f;

    [Header("Cantilever sag")]
    public float sagDuration = 10.0f;

    [Header("Cantilever release")]
    [Tooltip("The cantilever evolves freely before the free edge is lifted.")]
    public float cantileverSettleSeconds = 5.0f;
    public float cantileverLiftSeconds = 0.5f;
    public float cantileverHoldSeconds = 0.5f;
    public float cantileverRingDownSeconds = 5.0f;
    public float cantileverLiftAmplitude = 0.10f;

    [Header("Output")]
    [Tooltip("Created below Assets/Exports.")]
    public string outputFolderName = "Calibration";

    private readonly List<int> fixedEdge = new List<int>();
    private readonly List<int> movingEdge = new List<int>();
    private readonly HashSet<int> fixedEdgeSet = new HashSet<int>();
    private readonly HashSet<int> movingEdgeSet = new HashSet<int>();

    private List<Vector3> initialPositions;
    private List<Vector3> capturedMovingEdgePositions;

    private Vector3 stretchDirection;
    private Vector3 shearDirection;

    private StreamWriter writer;
    private float simulationTime;
    private bool movingEdgeReleased;
    private bool capturedDampingEdge;
    private bool finished;

    private string parameterName;
    private float parameterValue;
    private string outputPath;

    private float PulseReleaseTime
    {
        get
        {
            return pulseRestSeconds +
                   pulseMoveSeconds +
                   pulseHoldSeconds;
        }
    }

    private float DampingLiftStart
    {
        get { return cantileverSettleSeconds; }
    }

    private float DampingReleaseTime
    {
        get
        {
            return cantileverSettleSeconds +
                   cantileverLiftSeconds +
                   cantileverHoldSeconds;
        }
    }

    private float TotalDuration
    {
        get
        {
            switch (mode)
            {
                case CalibrationMode.StretchPulse:
                case CalibrationMode.ShearPulse:
                    return PulseReleaseTime + pulseFreeSeconds;

                case CalibrationMode.CantileverSag:
                    return sagDuration;

                case CalibrationMode.CantileverRelease:
                    return DampingReleaseTime +
                           cantileverRingDownSeconds;

                default:
                    return 5.0f;
            }
        }
    }

    private void Awake()
    {
        if (mode == CalibrationMode.CantileverSag ||
            mode == CalibrationMode.CantileverRelease)
        {
            cloth.kappa_bnd = 0.1f * cloth.kappa;
        }
        if (cloth == null)
        {
            Debug.LogError(
                "ClothildeCalibrationBenchmark: assign TriangleMesh."
            );
            enabled = false;
            return;
        }

        // Prevent TriangleMesh.LoadClothType() from replacing values chosen
        // in the Inspector for the calibration sweep.
        cloth.typeCloth = TriangleMesh.ClothType.Personalize;

        // The calibration must use the physical grid, not the optional
        // visualization smoothing applied to the exported Unity vertices.
        cloth.smooth = 0;
        cloth.profiling = false;
        cloth.testCusick = false;

        if (applyRecommendedIsolationSettings)
            ApplyIsolationSettingsBeforePythonInitialization();
    }

    private void ApplyIsolationSettingsBeforePythonInitialization()
    {
        switch (mode)
        {
            case CalibrationMode.StretchPulse:
            case CalibrationMode.ShearPulse:
                // Isolate in-plane material response.
                cloth.delta = 0.0f;       // disables gravity loading
                cloth.alpha = 0.0f;       // no physical damping
                cloth.kappa = 0.0f;       // no interior bending
                cloth.kappa_bnd = 0.0f;   // no boundary bending
                break;

            case CalibrationMode.CantileverSag:
                // Gravity-to-inertia ratio is correct when delta == rho.
                cloth.delta = cloth.rho;
                break;

            case CalibrationMode.CantileverRelease:
                cloth.delta = cloth.rho;
                break;
        }
    }

    private void Start()
    {
        if (!enabled)
            return;

        CultureInfo.DefaultThreadCurrentCulture =
            CultureInfo.InvariantCulture;
        CultureInfo.DefaultThreadCurrentUICulture =
            CultureInfo.InvariantCulture;

        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = 60;

        initialPositions = cloth.getTotalMeshPositions();

        int expectedCount =
            cloth.numVertexWidth * cloth.numVertexHeight;

        if (initialPositions == null ||
            initialPositions.Count != expectedCount)
        {
            Debug.LogError(
                "ClothildeCalibrationBenchmark: expected " +
                expectedCount + " physical nodes, received " +
                (initialPositions == null
                    ? 0
                    : initialPositions.Count) + "."
            );
            enabled = false;
            return;
        }

        BuildBoundaryIndices();
        ComputeMaterialDirections();
        SelectCalibrationParameter();
        OpenCsv();

        // Apply the first set of boundary conditions before the first
        // simulation Update() and record the reference configuration.
        ApplyControls(0.0f);
        WriteFrame(0.0f, initialPositions);
    }

    private void BuildBoundaryIndices()
    {
        fixedEdge.Clear();
        movingEdge.Clear();
        fixedEdgeSet.Clear();
        movingEdgeSet.Clear();

        int fixedX = 0;
        int movingX = cloth.numVertexWidth - 1;

        for (int y = 0; y < cloth.numVertexHeight; y++)
        {
            int fixedIndex = GridIndex(fixedX, y);
            int movingIndex = GridIndex(movingX, y);

            fixedEdge.Add(fixedIndex);
            movingEdge.Add(movingIndex);
            fixedEdgeSet.Add(fixedIndex);
            movingEdgeSet.Add(movingIndex);
        }
    }

    private void ComputeMaterialDirections()
    {
        Vector3 fixedMean = MeanPosition(initialPositions, fixedEdge);
        Vector3 movingMean = MeanPosition(initialPositions, movingEdge);

        stretchDirection = (movingMean - fixedMean).normalized;

        Vector3 edgeDirection =
            initialPositions[movingEdge[movingEdge.Count - 1]] -
            initialPositions[movingEdge[0]];

        // Remove any small numerical component in the stretching direction.
        edgeDirection -=
            Vector3.Dot(edgeDirection, stretchDirection) *
            stretchDirection;

        shearDirection = edgeDirection.normalized;

        if (stretchDirection.sqrMagnitude < 0.5f ||
            shearDirection.sqrMagnitude < 0.5f)
        {
            Debug.LogError(
                "ClothildeCalibrationBenchmark: could not infer " +
                "the material directions from the initial grid."
            );
            enabled = false;
        }
    }

    private void SelectCalibrationParameter()
    {
        switch (mode)
        {
            case CalibrationMode.StretchPulse:
                parameterName = "str";
                parameterValue = cloth.str;
                break;

            case CalibrationMode.ShearPulse:
                parameterName = "shr";
                parameterValue = cloth.shr;
                break;

            case CalibrationMode.CantileverSag:
                parameterName = "kappa";
                parameterValue = cloth.kappa;
                break;

            case CalibrationMode.CantileverRelease:
                parameterName = "alpha";
                parameterValue = cloth.alpha;
                break;
        }
    }

    private void OpenCsv()
    {
        string directory = Path.Combine(
            Application.dataPath,
            "Exports",
            outputFolderName
        );

        Directory.CreateDirectory(directory);

        string safeParameter =
            parameterValue.ToString(
                "0.########E+0",
                CultureInfo.InvariantCulture
            ).Replace("+", "p").Replace("-", "m");

        string fileName = string.Format(
            CultureInfo.InvariantCulture,
            "{0}_{1}_{2}_{3}_{4:yyyyMMdd_HHmmss}.csv",
            runName,
            mode,
            parameterName,
            safeParameter,
            DateTime.Now
        );

        outputPath = Path.Combine(directory, fileName);
        writer = new StreamWriter(outputPath, false);

        writer.WriteLine(
            "simulator,experiment,parameter_name,parameter_value," +
            "time_s,phase,node_index,u,v,x,y,z,controlled," +
            "str,shr,kappa,kappa_bnd,alpha,rho,delta,dt,sub_steps"
        );
    }

    private void Update()
    {
        if (!enabled || finished)
            return;

        float nextTime = simulationTime + cloth.dt;
        ApplyControls(nextTime);
    }

    private void LateUpdate()
    {
        if (!enabled || finished)
            return;

        simulationTime += cloth.dt;

        List<Vector3> currentPositions =
            cloth.getTotalMeshPositions();

        if (currentPositions == null ||
            currentPositions.Count != initialPositions.Count)
        {
            Debug.LogError(
                "ClothildeCalibrationBenchmark: node count changed."
            );
            FinishExperiment();
            return;
        }

        WriteFrame(simulationTime, currentPositions);

        if (simulationTime + 1e-6f >= TotalDuration)
            FinishExperiment();
    }

    private void ApplyControls(float time)
    {
        // The cantilever root / in-plane fixed edge remains fixed
        // throughout all four experiments.
        foreach (int index in fixedEdge)
        {
            cloth.DragHandle(
                index,
                initialPositions[index]
            );
        }

        switch (mode)
        {
            case CalibrationMode.StretchPulse:
                ApplyPulseControl(
                    time,
                    stretchDirection,
                    stretchAmplitude
                );
                break;

            case CalibrationMode.ShearPulse:
                ApplyPulseControl(
                    time,
                    shearDirection,
                    shearAmplitude
                );
                break;

            case CalibrationMode.CantileverSag:
                // Only the root edge is fixed.
                ReleaseMovingEdgeOnce();
                break;

            case CalibrationMode.CantileverRelease:
                ApplyCantileverReleaseControl(time);
                break;
        }
    }

    private void ApplyPulseControl(
        float time,
        Vector3 direction,
        float amplitude)
    {
        if (time <= PulseReleaseTime)
        {
            float offset =
                amplitude * PulseSmoothFraction(time);

            for (int i = 0; i < movingEdge.Count; i++)
            {
                int index = movingEdge[i];

                cloth.DragHandle(
                    index,
                    initialPositions[index] +
                    direction * offset
                );
            }
        }
        else
        {
            ReleaseMovingEdgeOnce();
        }
    }

    private float PulseSmoothFraction(float time)
    {
        if (time <= pulseRestSeconds)
            return 0.0f;

        float moveEnd =
            pulseRestSeconds + pulseMoveSeconds;

        if (time < moveEnd)
        {
            float t =
                (time - pulseRestSeconds) /
                Mathf.Max(pulseMoveSeconds, 1e-6f);

            // Smoothstep gives zero target velocity at both ends.
            return t * t * (3.0f - 2.0f * t);
        }

        return 1.0f;
    }

    private void ApplyCantileverReleaseControl(float time)
    {
        if (time < DampingLiftStart)
        {
            // The free edge must remain unconstrained while the
            // cantilever approaches its equilibrium configuration.
            ReleaseMovingEdgeOnce();
            return;
        }

        if (!capturedDampingEdge)
        {
            List<Vector3> positions =
                cloth.getTotalMeshPositions();

            capturedMovingEdgePositions =
                new List<Vector3>(movingEdge.Count);

            foreach (int index in movingEdge)
                capturedMovingEdgePositions.Add(positions[index]);

            capturedDampingEdge = true;
            movingEdgeReleased = false;
        }

        if (time <= DampingReleaseTime)
        {
            float liftFraction =
                CantileverLiftFraction(time);

            Vector3 offset =
                Vector3.up *
                cantileverLiftAmplitude *
                liftFraction;

            for (int i = 0; i < movingEdge.Count; i++)
            {
                cloth.DragHandle(
                    movingEdge[i],
                    capturedMovingEdgePositions[i] + offset
                );
            }
        }
        else
        {
            ReleaseMovingEdgeOnce();
        }
    }

    private float CantileverLiftFraction(float time)
    {
        float liftEnd =
            cantileverSettleSeconds +
            cantileverLiftSeconds;

        if (time < liftEnd)
        {
            float t =
                (time - cantileverSettleSeconds) /
                Mathf.Max(cantileverLiftSeconds, 1e-6f);

            t = Mathf.Clamp01(t);
            return t * t * (3.0f - 2.0f * t);
        }

        return 1.0f;
    }

    private void ReleaseMovingEdgeOnce()
    {
        if (movingEdgeReleased)
            return;

        foreach (int index in movingEdge)
            cloth.LetHandleGo(index);

        movingEdgeReleased = true;
    }

    private bool IsControlled(int nodeIndex, float time)
    {
        if (fixedEdgeSet.Contains(nodeIndex))
            return true;

        if (!movingEdgeSet.Contains(nodeIndex))
            return false;

        switch (mode)
        {
            case CalibrationMode.StretchPulse:
            case CalibrationMode.ShearPulse:
                return time <= PulseReleaseTime;

            case CalibrationMode.CantileverSag:
                return false;

            case CalibrationMode.CantileverRelease:
                return time >= DampingLiftStart &&
                       time <= DampingReleaseTime;

            default:
                return false;
        }
    }

    private string CurrentPhase(float time)
    {
        switch (mode)
        {
            case CalibrationMode.StretchPulse:
            case CalibrationMode.ShearPulse:
                if (time < pulseRestSeconds)
                    return "rest";

                if (time <
                    pulseRestSeconds + pulseMoveSeconds)
                {
                    return "move";
                }

                if (time <= PulseReleaseTime)
                    return "hold";

                return "free";

            case CalibrationMode.CantileverSag:
                return "sag";

            case CalibrationMode.CantileverRelease:
                if (time < cantileverSettleSeconds)
                    return "settle";

                if (time <
                    cantileverSettleSeconds +
                    cantileverLiftSeconds)
                {
                    return "lift";
                }

                if (time <= DampingReleaseTime)
                    return "hold";

                return "ringdown";

            default:
                return "unknown";
        }
    }

    private void WriteFrame(
        float time,
        List<Vector3> positions)
    {
        string phase = CurrentPhase(time);

        for (int x = 0; x < cloth.numVertexWidth; x++)
        {
            float u =
                x / (float)(cloth.numVertexWidth - 1);

            for (int y = 0; y < cloth.numVertexHeight; y++)
            {
                float v =
                    y / (float)(cloth.numVertexHeight - 1);

                int index = GridIndex(x, y);
                Vector3 p = positions[index];

                writer.WriteLine(string.Format(
                    CultureInfo.InvariantCulture,
                    "Clothilde,{0},{1},{2:R},{3:F6},{4}," +
                    "{5},{6:F6},{7:F6},{8:F9},{9:F9},{10:F9}," +
                    "{11},{12:R},{13:R},{14:R},{15:R},{16:R}," +
                    "{17:R},{18:R},{19:R},{20}",
                    mode,
                    parameterName,
                    parameterValue,
                    time,
                    phase,
                    index,
                    u,
                    v,
                    p.x,
                    p.y,
                    p.z,
                    IsControlled(index, time) ? 1 : 0,
                    cloth.str,
                    cloth.shr,
                    cloth.kappa,
                    cloth.kappa_bnd,
                    cloth.alpha,
                    cloth.rho,
                    cloth.delta,
                    cloth.dt,
                    cloth.sub_steps
                ));
            }
        }
    }

    private int GridIndex(int x, int y)
    {
        return x * cloth.numVertexHeight + y;
    }

    private Vector3 MeanPosition(
        List<Vector3> positions,
        List<int> indices)
    {
        Vector3 sum = Vector3.zero;

        foreach (int index in indices)
            sum += positions[index];

        return sum / indices.Count;
    }

    private void FinishExperiment()
    {
        if (finished)
            return;

        finished = true;

        foreach (int index in fixedEdge)
            cloth.LetHandleGo(index);

        foreach (int index in movingEdge)
            cloth.LetHandleGo(index);

        if (writer != null)
        {
            writer.Flush();
            writer.Close();
            writer = null;
        }

        Debug.Log(
            "Clothilde calibration CSV saved to:\n" +
            outputPath
        );

#if UNITY_EDITOR
        if (stopPlayModeWhenFinished)
            UnityEditor.EditorApplication.isPlaying = false;
#endif
    }

    private void OnDestroy()
    {
        if (writer != null)
        {
            writer.Flush();
            writer.Close();
            writer = null;
        }
    }
}
