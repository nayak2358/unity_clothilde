using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

/// <summary>
/// Deterministic one-corner self-collision benchmarks for Clothilde.
///
/// CornerFoldCompression:
///   - leaves the entire cloth free except for one controlled corner,
///   - moves only the (u=1,v=1) corner through a 180-degree arc,
///   - lowers the corner toward an interior target to create self-contact,
///   - holds the configuration.
///   Use this mode to fit "slf". Self-friction is set to zero automatically.
///
/// CornerFoldSlide:
///   - repeats the same fold/compression preparation,
///   - then drags the same corner tangentially along material v.
///   Use this mode to fit "mu_s" after freezing the fitted slf.
///
/// All previously calibrated material parameters (str, shr, kappa, alpha)
/// remain unchanged. Gravity is disabled to avoid table/floor contact.
/// </summary>
[DefaultExecutionOrder(-1000)]
public class ClothildeCornerSelfCollisionBenchmark : MonoBehaviour
{
    public enum BenchmarkMode
    {
        CornerFoldCompression,
        CornerFoldSlide
    }

    [Header("Reference")]
    public TriangleMesh cloth;

    [Header("Experiment")]
    public BenchmarkMode mode = BenchmarkMode.CornerFoldCompression;
    public string runName = "clothilde_corner_self_collision";
    public bool stopPlayModeWhenFinished = true;

    [Header("Timing")]
    public float restSeconds = 0.50f;
    public float foldSeconds = 2.50f;
    public float compressionSeconds = 0.75f;
    public float compressionHoldSeconds = 1.50f;
    public float preSlideHoldSeconds = 0.75f;
    public float slideSeconds = 2.00f;
    public float slideHoldSeconds = 1.00f;

    [Header("Fold target in material coordinates")]
    [Range(0.0f, 1.0f)]
    public float targetU = 0.25f;

    [Range(0.0f, 1.0f)]
    public float targetV = 0.25f;

    [Tooltip(
        "Corner height above the target material point at the end of the " +
        "180-degree fold. Keep this slightly larger than the nominal " +
        "self-contact distance."
    )]
    public float approachGap = 0.045f;

    [Tooltip(
        "Corner height used to preload self-contact. For the 0.035 m grid " +
        "and thck=1.025, the nominal two-particle contact distance is about " +
        "0.035 m; 0.025 m provides a moderate compression."
    )]
    public float compressedGap = 0.025f;

    [Header("Tangential slide")]
    [Tooltip(
        "Tangential movement of the controlled corner after contact is made."
    )]
    public float slideDistance = 0.120f;

    [Header("Output")]
    public string outputFolderName = "CornerSelfCollisionCalibration";

    private readonly List<int> fixedEdge = new List<int>();
    private readonly HashSet<int> controlledNodes = new HashSet<int>();

    private List<Vector3> initialPositions;

    private int movingCornerIndex;
    private int targetNodeIndex;

    private Vector3 materialUDirection;
    private Vector3 materialVDirection;
    private Vector3 materialNormal;

    private Vector3 movingCornerInitial;
    private Vector3 targetPointInitial;

    private StreamWriter writer;
    private float simulationTime;
    private bool finished;
    private string outputPath;

    private string parameterName;
    private float parameterValue;

    private float FoldStart => restSeconds;
    private float FoldEnd => FoldStart + foldSeconds;
    private float CompressionEnd => FoldEnd + compressionSeconds;

    private float SlideStart =>
        CompressionEnd + preSlideHoldSeconds;

    private float SlideEnd =>
        SlideStart + slideSeconds;

    private float TotalDuration
    {
        get
        {
            if (mode == BenchmarkMode.CornerFoldCompression)
                return CompressionEnd + compressionHoldSeconds;

            return SlideEnd + slideHoldSeconds;
        }
    }

    private void Awake()
    {
        if (cloth == null)
        {
            Debug.LogError(
                "ClothildeCornerSelfCollisionBenchmark: assign TriangleMesh."
            );
            enabled = false;
            return;
        }

        // Preserve Inspector values rather than loading a predefined cloth.
        cloth.typeCloth = TriangleMesh.ClothType.Personalize;

        // Export and compare physical nodes, not smoothed visual nodes.
        cloth.smooth = 0;
        cloth.profiling = false;
        cloth.testCusick = false;

        // No gravity, table, or floor loading in this calibration.
        cloth.delta = 0.1f;

        if (mode == BenchmarkMode.CornerFoldCompression)
        {
            // Isolate normal self-contact response.
            cloth.mu_s = 0.0f;
            parameterName = "slf";
            parameterValue = cloth.slf;
        }
        else
        {
            // The fitted slf must already be set in TriangleMesh.
            parameterName = "mu_s";
            parameterValue = cloth.mu_s;
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
                "Expected " + expectedCount + " physical nodes, received " +
                (initialPositions == null ? 0 : initialPositions.Count) + "."
            );
            enabled = false;
            return;
        }

        BuildIndices();
        ComputeMaterialFrame();

        movingCornerInitial =
            initialPositions[movingCornerIndex];

        targetPointInitial =
            initialPositions[targetNodeIndex];

        OpenCsv();

        ApplyControls(0.0f);
        WriteFrame(0.0f, initialPositions);
    }

    private void BuildIndices()
    {
        fixedEdge.Clear();
        controlledNodes.Clear();

        int lastX = cloth.numVertexWidth - 1;
        int lastY = cloth.numVertexHeight - 1;

        // Keep the u=0 material edge only as a geometric reference
        // for constructing the initial material frame.
        //
        // IMPORTANT:
        // these nodes are NOT constrained and are NOT marked as controlled.
        for (int y = 0; y < cloth.numVertexHeight; y++)
        {
            int index = GridIndex(0, y);
            fixedEdge.Add(index);
        }

        // Move only one corner.
        movingCornerIndex = GridIndex(lastX, lastY);
        controlledNodes.Add(movingCornerIndex);

        int targetX = Mathf.Clamp(
            Mathf.RoundToInt(targetU * lastX),
            1,
            lastX - 1
        );

        int targetY = Mathf.Clamp(
            Mathf.RoundToInt(targetV * lastY),
            1,
            lastY - 1
        );

        targetNodeIndex = GridIndex(targetX, targetY);
    }

    private void ComputeMaterialFrame()
    {
        int lastX = cloth.numVertexWidth - 1;
        int lastY = cloth.numVertexHeight - 1;

        Vector3 u0Mean = Vector3.zero;
        Vector3 u1Mean = Vector3.zero;

        for (int y = 0; y < cloth.numVertexHeight; y++)
        {
            u0Mean += initialPositions[GridIndex(0, y)];
            u1Mean += initialPositions[GridIndex(lastX, y)];
        }

        u0Mean /= cloth.numVertexHeight;
        u1Mean /= cloth.numVertexHeight;

        materialUDirection =
            (u1Mean - u0Mean).normalized;

        Vector3 vRaw =
            initialPositions[GridIndex(0, lastY)] -
            initialPositions[GridIndex(0, 0)];

        vRaw -=
            Vector3.Dot(vRaw, materialUDirection) *
            materialUDirection;

        materialVDirection = vRaw.normalized;

        materialNormal = Vector3.Cross(
            materialUDirection,
            materialVDirection
        ).normalized;

        // Both benchmark scenes start with a horizontal cloth.
        if (Vector3.Dot(materialNormal, Vector3.up) < 0.0f)
            materialNormal = -materialNormal;

        if (materialUDirection.sqrMagnitude < 0.5f ||
            materialVDirection.sqrMagnitude < 0.5f ||
            materialNormal.sqrMagnitude < 0.5f)
        {
            Debug.LogError(
                "Could not infer a valid material frame."
            );
            enabled = false;
        }
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

        List<Vector3> positions =
            cloth.getTotalMeshPositions();

        if (positions == null ||
            positions.Count != initialPositions.Count)
        {
            Debug.LogError("Physical node count changed.");
            FinishExperiment();
            return;
        }

        WriteFrame(simulationTime, positions);

        if (simulationTime + 1e-6f >= TotalDuration)
            FinishExperiment();
    }

    private void ApplyControls(float time)
    {
        // ONLY ONE PARTICLE IS KINEMATICALLY CONTROLLED:
        // the material corner (u,v) = (1,1).
        //
        // The u=0 edge is NOT fixed. It is stored in fixedEdge only
        // to construct the initial material coordinate frame.
        cloth.DragHandle(
            movingCornerIndex,
            MovingCornerTarget(time)
        );
    }

    private Vector3 MovingCornerTarget(float time)
    {
        if (time <= FoldStart)
            return movingCornerInitial;

        if (time < FoldEnd)
        {
            float fraction = SmoothStep01(
                (time - FoldStart) /
                Mathf.Max(foldSeconds, 1e-6f)
            );

            return SemicircularFoldTarget(fraction);
        }

        float gap = CurrentGap(time);
        float slideOffset = CurrentSlideOffset(time);

        return
            targetPointInitial +
            materialNormal * gap +
            materialVDirection * slideOffset;
    }

    private Vector3 SemicircularFoldTarget(float fraction)
    {
        Vector3 center =
            0.5f * (movingCornerInitial + targetPointInitial);

        Vector3 radial =
            0.5f * (movingCornerInitial - targetPointInitial);

        float radius = radial.magnitude;
        float theta = Mathf.PI * fraction;

        // Starts at the original corner and ends above targetPointInitial.
        // The arc is lifted along the initial cloth normal.
        float gapFraction =
            0.5f * (1.0f - Mathf.Cos(theta));

        return
            center +
            Mathf.Cos(theta) * radial +
            Mathf.Sin(theta) * radius * materialNormal +
            gapFraction * approachGap * materialNormal;
    }

    private float CurrentGap(float time)
    {
        if (time <= FoldEnd)
            return approachGap;

        if (time >= CompressionEnd)
            return compressedGap;

        float fraction = SmoothStep01(
            (time - FoldEnd) /
            Mathf.Max(compressionSeconds, 1e-6f)
        );

        return Mathf.Lerp(
            approachGap,
            compressedGap,
            fraction
        );
    }

    private float CurrentSlideOffset(float time)
    {
        if (mode != BenchmarkMode.CornerFoldSlide ||
            time <= SlideStart)
        {
            return 0.0f;
        }

        if (time >= SlideEnd)
            return slideDistance;

        float fraction = SmoothStep01(
            (time - SlideStart) /
            Mathf.Max(slideSeconds, 1e-6f)
        );

        return slideDistance * fraction;
    }

    private float SmoothStep01(float value)
    {
        float t = Mathf.Clamp01(value);
        return t * t * (3.0f - 2.0f * t);
    }

    private string CurrentPhase(float time)
    {
        if (time < FoldStart)
            return "rest";

        if (time < FoldEnd)
            return "fold";

        if (time < CompressionEnd)
            return "compress";

        if (mode == BenchmarkMode.CornerFoldCompression)
            return "hold";

        if (time < SlideStart)
            return "prehold";

        if (time < SlideEnd)
            return "slide";

        return "hold";
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
            "slf,mu_s,thck,str,shr,kappa,kappa_bnd,alpha,rho,delta," +
            "dt,sub_steps,target_u,target_v,approach_gap,compressed_gap," +
            "slide_distance"
        );
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
                    "{17:R},{18:R},{19:R},{20:R},{21:R},{22:R}," +
                    "{23},{24:R},{25:R},{26:R},{27:R},{28:R}",
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
                    controlledNodes.Contains(index) ? 1 : 0,
                    cloth.slf,
                    cloth.mu_s,
                    cloth.thck,
                    cloth.str,
                    cloth.shr,
                    cloth.kappa,
                    cloth.kappa_bnd,
                    cloth.alpha,
                    cloth.rho,
                    cloth.delta,
                    cloth.dt,
                    cloth.sub_steps,
                    targetU,
                    targetV,
                    approachGap,
                    compressedGap,
                    slideDistance
                ));
            }
        }
    }

    private int GridIndex(int x, int y)
    {
        return x * cloth.numVertexHeight + y;
    }

    private void FinishExperiment()
    {
        if (finished)
            return;

        finished = true;

        foreach (int index in controlledNodes)
            cloth.LetHandleGo(index);

        if (writer != null)
        {
            writer.Flush();
            writer.Close();
            writer = null;
        }

        Debug.Log(
            "Clothilde corner self-collision CSV saved to:\n" +
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
