using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

/// <summary>
/// Runs and exports the non-VR Clothilde experiments:
/// 1. EdgeShear: fixes three nodes on the left edge and translates
///    three corresponding nodes on the right edge.
/// 2. TableSlide: applies no controls and only records the cloth motion.
///
/// Replace the previous Exporter.cs with this file.
/// Add the component to an empty GameObject and assign TriangleMesh.
/// </summary>
[DefaultExecutionOrder(-100)]
public class Exporter : MonoBehaviour
{
    public enum ExperimentType
    {
        EdgeShear,
        TableSlide
    }

    [Header("References")]
    public TriangleMesh cloth;

    [Header("Experiment")]
    public ExperimentType experiment = ExperimentType.EdgeShear;
    public string runName = "";
    public bool autoStart = true;
    public KeyCode startStopKey = KeyCode.E;

    [Header("Edge shear")]
    public float settleSeconds = 1.0f;
    public float moveSeconds = 2.0f;
    public float holdSeconds = 2.0f;
    public float pullDistance = 0.15f;
    public Vector3 pullDirection = Vector3.forward;

    [Header("Table slide")]
    public float tableSlideDuration = 6.0f;

    [Header("Output")]
    [Tooltip("Created inside Assets/Exports.")]
    public string outputFolderName = "NonVRExperiments";

    private StreamWriter metricsWriter;
    private StreamWriter positionsWriter;

    private string sessionFolder;

    private List<Vector3> controlReferencePositions;
    private List<Vector3> previousPositions;

    private Vector3 metricReferenceCenter;
    private float metricReferenceAngle;
    private bool metricReferenceCaptured;

    private readonly List<float> sampleTimes = new List<float>();
    private readonly List<float> angleChanges = new List<float>();
    private readonly List<float> comDisplacements = new List<float>();
    private readonly List<float> rmsSpeeds = new List<float>();

    private float simulationTime;
    private bool running;
    private bool finished;

    private float TotalDuration
    {
        get
        {
            if (experiment == ExperimentType.TableSlide)
                return tableSlideDuration;

            return settleSeconds + moveSeconds + holdSeconds;
        }
    }

    private void Start()
    {
        CultureInfo.DefaultThreadCurrentCulture = CultureInfo.InvariantCulture;
        CultureInfo.DefaultThreadCurrentUICulture = CultureInfo.InvariantCulture;

        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = 60;

        if (cloth == null)
            cloth = FindFirstObjectByType<TriangleMesh>();

        if (cloth == null)
        {
            Debug.LogError(
                "Exporter: no TriangleMesh was found. " +
                "Assign the cloth field in the Inspector."
            );
            enabled = false;
            return;
        }

        if (autoStart)
            BeginExperiment();
    }

    private void Update()
    {
        if (Input.GetKeyDown(startStopKey))
        {
            if (!running && !finished)
            {
                BeginExperiment();
            }
            else if (running)
            {
                FinishExperiment();
            }
        }

        if (!running)
            return;

        // TriangleMesh advances one fixed simulator frame per Unity Update.
        float nextSimulationTime = simulationTime + cloth.dt;

        if (experiment == ExperimentType.EdgeShear)
        {
            // Capture the undeformed reference after the settling interval,
            // immediately before the prescribed edge motion begins.
            if (!metricReferenceCaptured &&
                nextSimulationTime >= settleSeconds)
            {
                CaptureMetricReference(cloth.getTotalMeshPositions());
            }

            ApplyEdgeShear(nextSimulationTime);
        }
    }

    private void LateUpdate()
    {
        if (!running)
            return;

        simulationTime += cloth.dt;

        List<Vector3> currentPositions = cloth.getTotalMeshPositions();

        if (currentPositions == null ||
            currentPositions.Count != cloth.numVertexWidth * cloth.numVertexHeight)
        {
            Debug.LogError("Exporter: unexpected physical-node count.");
            FinishExperiment();
            return;
        }

        WriteSample(simulationTime, currentPositions);
        previousPositions = new List<Vector3>(currentPositions);

        if (simulationTime + 1e-6f >= TotalDuration)
            FinishExperiment();
    }

    private void BeginExperiment()
    {
        if (running || finished)
            return;

        List<Vector3> positions = cloth.getTotalMeshPositions();

        if (positions == null || positions.Count == 0)
        {
            Debug.LogError("Exporter: cloth positions are not available.");
            return;
        }

        string experimentName = experiment.ToString();
        string safeRunName = string.IsNullOrWhiteSpace(runName)
            ? experimentName
            : runName.Trim();

        string rootFolder = Path.Combine(
            Application.dataPath,
            "Exports",
            outputFolderName
        );

        sessionFolder = Path.Combine(
            rootFolder,
            safeRunName + "_" + DateTime.Now.ToString("yyyyMMdd_HHmmss")
        );

        Directory.CreateDirectory(sessionFolder);

        metricsWriter = new StreamWriter(
            Path.Combine(sessionFolder, "Metrics.csv"),
            false
        );

        positionsWriter = new StreamWriter(
            Path.Combine(sessionFolder, "MeshPositions.csv"),
            false
        );

        metricsWriter.WriteLine(
            "simulator,experiment,phase,time_s,particle_count," +
            "target_offset_m,com_x,com_y,com_z," +
            "com_horizontal_displacement_m,center_angle_deg," +
            "center_angle_change_deg,rms_speed_mps"
        );

        positionsWriter.WriteLine(
            "simulator,experiment,time_s,node_index,x,y,z"
        );

        controlReferencePositions = new List<Vector3>(positions);
        previousPositions = new List<Vector3>(positions);

        sampleTimes.Clear();
        angleChanges.Clear();
        comDisplacements.Clear();
        rmsSpeeds.Clear();

        simulationTime = 0.0f;
        metricReferenceCaptured = false;

        if (experiment == ExperimentType.TableSlide)
            CaptureMetricReference(positions);

        WriteFaces();
        WriteParameters();
        WriteControls();
        WriteSample(0.0f, positions);

        running = true;

        Debug.Log(
            "Exporter: " + experimentName + " started. " +
            "The CSV files will be saved automatically to:\n" +
            sessionFolder
        );
    }

    private void ApplyEdgeShear(float time)
    {
        float targetOffset = CurrentTargetOffset(time);

        Vector3 direction = pullDirection.sqrMagnitude > 0.0f
            ? pullDirection.normalized
            : Vector3.forward;

        int width = cloth.numVertexWidth;
        int height = cloth.numVertexHeight;

        // Match the six-point setup used for the Obi comparison:
        // bottom, middle, and top on each side.
        int[] rows = { 0, height / 2, height - 1 };

        foreach (int row in rows)
        {
            int leftNode = GridIndex(0, row);
            int rightNode = GridIndex(width - 1, row);

            cloth.DragHandle(
                leftNode,
                controlReferencePositions[leftNode]
            );

            cloth.DragHandle(
                rightNode,
                controlReferencePositions[rightNode] +
                direction * targetOffset
            );
        }
    }

    private void ReleaseEdgeControls()
    {
        if (experiment != ExperimentType.EdgeShear)
            return;

        int width = cloth.numVertexWidth;
        int height = cloth.numVertexHeight;
        int[] rows = { 0, height / 2, height - 1 };

        foreach (int row in rows)
        {
            cloth.LetHandleGo(GridIndex(0, row));
            cloth.LetHandleGo(GridIndex(width - 1, row));
        }
    }

    private float CurrentTargetOffset(float time)
    {
        if (experiment != ExperimentType.EdgeShear)
            return 0.0f;

        float normalizedTime = Mathf.Clamp01(
            (time - settleSeconds) / Mathf.Max(moveSeconds, 1e-6f)
        );

        // Smoothstep: zero velocity at the beginning and end.
        normalizedTime =
            normalizedTime * normalizedTime *
            (3.0f - 2.0f * normalizedTime);

        return pullDistance * normalizedTime;
    }

    private string CurrentPhase(float time)
    {
        if (experiment == ExperimentType.TableSlide)
            return "slide";

        if (time < settleSeconds)
            return "settle";

        if (time < settleSeconds + moveSeconds)
            return "move";

        return "hold";
    }

    private void CaptureMetricReference(List<Vector3> positions)
    {
        metricReferenceCenter = Average(positions);
        metricReferenceAngle = CenterAngle(positions);
        metricReferenceCaptured = true;
    }

    private void WriteSample(
        float time,
        List<Vector3> positions)
    {
        Vector3 center = Average(positions);
        float centerAngle = CenterAngle(positions);

        float centerDisplacement = float.NaN;
        float centerAngleChange = float.NaN;

        if (metricReferenceCaptured)
        {
            centerDisplacement = HorizontalDistance(
                center,
                metricReferenceCenter
            );

            centerAngleChange = Mathf.Abs(
                centerAngle - metricReferenceAngle
            );
        }

        float sampleDt;

        if (sampleTimes.Count == 0)
            sampleDt = cloth.dt;
        else
            sampleDt = Mathf.Max(
                time - sampleTimes[sampleTimes.Count - 1],
                1e-6f
            );

        float rmsSpeed = RmsSpeed(
            positions,
            previousPositions,
            sampleDt
        );

        float targetOffset = CurrentTargetOffset(time);

        metricsWriter.WriteLine(string.Format(
            CultureInfo.InvariantCulture,
            "Clothilde,{0},{1},{2:F6},{3},{4:F6}," +
            "{5:F6},{6:F6},{7:F6},{8}," +
            "{9:F6},{10},{11:F6}",
            experiment,
            CurrentPhase(time),
            time,
            positions.Count,
            targetOffset,
            center.x,
            center.y,
            center.z,
            CsvNumber(centerDisplacement),
            centerAngle,
            CsvNumber(centerAngleChange),
            rmsSpeed
        ));

        for (int i = 0; i < positions.Count; i++)
        {
            Vector3 p = positions[i];

            positionsWriter.WriteLine(string.Format(
                CultureInfo.InvariantCulture,
                "Clothilde,{0},{1:F6},{2},{3:F6},{4:F6},{5:F6}",
                experiment,
                time,
                i,
                p.x,
                p.y,
                p.z
            ));
        }

        sampleTimes.Add(time);
        angleChanges.Add(centerAngleChange);
        comDisplacements.Add(centerDisplacement);
        rmsSpeeds.Add(rmsSpeed);
    }

    private void WriteFaces()
    {
        int[,] faces = cloth.getFaces();

        using (StreamWriter writer = new StreamWriter(
            Path.Combine(sessionFolder, "Faces.csv"),
            false))
        {
            writer.WriteLine("face_index,n0,n1,n2,n3");

            for (int i = 0; i < faces.GetLength(0); i++)
            {
                writer.WriteLine(string.Format(
                    CultureInfo.InvariantCulture,
                    "{0},{1},{2},{3},{4}",
                    i,
                    faces[i, 0],
                    faces[i, 1],
                    faces[i, 2],
                    faces[i, 3]
                ));
            }
        }
    }

    private void WriteControls()
    {
        using (StreamWriter writer = new StreamWriter(
            Path.Combine(sessionFolder, "Controls.csv"),
            false))
        {
            writer.WriteLine(
                "role,node_index,initial_x,initial_y,initial_z"
            );

            if (experiment != ExperimentType.EdgeShear)
                return;

            int width = cloth.numVertexWidth;
            int height = cloth.numVertexHeight;
            int[] rows = { 0, height / 2, height - 1 };

            foreach (int row in rows)
            {
                WriteControlRow(
                    writer,
                    "fixed_left",
                    GridIndex(0, row)
                );

                WriteControlRow(
                    writer,
                    "moving_right",
                    GridIndex(width - 1, row)
                );
            }
        }
    }

    private void WriteControlRow(
        StreamWriter writer,
        string role,
        int nodeIndex)
    {
        Vector3 p = controlReferencePositions[nodeIndex];

        writer.WriteLine(string.Format(
            CultureInfo.InvariantCulture,
            "{0},{1},{2:F6},{3:F6},{4:F6}",
            role,
            nodeIndex,
            p.x,
            p.y,
            p.z
        ));
    }

    private void WriteParameters()
    {
        using (StreamWriter writer = new StreamWriter(
            Path.Combine(sessionFolder, "Parameters.csv"),
            false))
        {
            writer.WriteLine("parameter,value");

            WriteParameter(writer, "simulator", "Clothilde");
            WriteParameter(writer, "experiment", experiment.ToString());
            WriteParameter(writer, "date", DateTime.Now.ToString("O"));
            WriteParameter(writer, "num_vertex_width", cloth.numVertexWidth);
            WriteParameter(writer, "num_vertex_height", cloth.numVertexHeight);
            WriteParameter(writer, "grid_width_m", cloth.gridWidth);
            WriteParameter(writer, "grid_height_m", cloth.gridHeight);
            WriteParameter(writer, "dt_s", cloth.dt);
            WriteParameter(writer, "sub_steps", cloth.sub_steps);
            WriteParameter(writer, "density", cloth.rho);
            WriteParameter(writer, "damping", cloth.alpha);
            WriteParameter(writer, "bending_compliance", cloth.kappa);
            WriteParameter(writer, "boundary_bending_compliance", cloth.kappa_bnd);
            WriteParameter(writer, "stretch_compliance", cloth.str);
            WriteParameter(writer, "shear_compliance", cloth.shr);
            WriteParameter(writer, "external_friction", cloth.mu_f);
            WriteParameter(writer, "self_friction", cloth.mu_s);
            WriteParameter(writer, "thickness", cloth.thck);
            WriteParameter(writer, "self_collision_compliance", cloth.slf);
            WriteParameter(writer, "total_duration_s", TotalDuration);

            if (experiment == ExperimentType.EdgeShear)
            {
                WriteParameter(writer, "settle_seconds", settleSeconds);
                WriteParameter(writer, "move_seconds", moveSeconds);
                WriteParameter(writer, "hold_seconds", holdSeconds);
                WriteParameter(writer, "pull_distance_m", pullDistance);

                Vector3 direction = pullDirection.sqrMagnitude > 0.0f
                    ? pullDirection.normalized
                    : Vector3.forward;

                WriteParameter(writer, "pull_direction_x", direction.x);
                WriteParameter(writer, "pull_direction_y", direction.y);
                WriteParameter(writer, "pull_direction_z", direction.z);
            }
        }
    }

    private void WriteParameter(
        StreamWriter writer,
        string parameter,
        object value)
    {
        writer.WriteLine(
            parameter + "," +
            Convert.ToString(value, CultureInfo.InvariantCulture)
        );
    }

    private void WriteSummary()
    {
        float finalComDisplacement =
            LastFiniteValue(comDisplacements);

        float finalAngleChange =
            LastFiniteValue(angleChanges);

        float maximumComDisplacement =
            MaximumFiniteValue(comDisplacements);

        float maximumAngleChange =
            MaximumFiniteValue(angleChanges);

        float finalSecondStart =
            Mathf.Max(0.0f, TotalDuration - 1.0f);

        float finalSecondRmsSum = 0.0f;
        int finalSecondSampleCount = 0;

        for (int i = 0; i < sampleTimes.Count; i++)
        {
            if (sampleTimes[i] >= finalSecondStart)
            {
                finalSecondRmsSum += rmsSpeeds[i];
                finalSecondSampleCount++;
            }
        }

        float finalSecondMeanRms =
            finalSecondSampleCount > 0
                ? finalSecondRmsSum / finalSecondSampleCount
                : float.NaN;

        using (StreamWriter writer = new StreamWriter(
            Path.Combine(sessionFolder, "Summary.csv"),
            false))
        {
            writer.WriteLine(
                "simulator,experiment,particle_count,duration_s," +
                "final_target_offset_m," +
                "final_com_horizontal_displacement_m," +
                "max_com_horizontal_displacement_m," +
                "final_center_angle_change_deg," +
                "max_center_angle_change_deg," +
                "final_second_mean_rms_speed_mps"
            );

            writer.WriteLine(string.Format(
                CultureInfo.InvariantCulture,
                "Clothilde,{0},{1},{2:F6},{3:F6}," +
                "{4},{5},{6},{7},{8}",
                experiment,
                cloth.numVertexWidth * cloth.numVertexHeight,
                simulationTime,
                CurrentTargetOffset(simulationTime),
                CsvNumber(finalComDisplacement),
                CsvNumber(maximumComDisplacement),
                CsvNumber(finalAngleChange),
                CsvNumber(maximumAngleChange),
                CsvNumber(finalSecondMeanRms)
            ));
        }
    }

    private void FinishExperiment()
    {
        if (!running)
            return;

        running = false;
        finished = true;

        ReleaseEdgeControls();

        if (metricsWriter != null)
        {
            metricsWriter.Flush();
            metricsWriter.Close();
            metricsWriter = null;
        }

        if (positionsWriter != null)
        {
            positionsWriter.Flush();
            positionsWriter.Close();
            positionsWriter = null;
        }

        WriteSummary();

        Debug.Log(
            "Exporter: experiment finished. CSV files saved to:\n" +
            sessionFolder
        );
    }

    private Vector3 Average(List<Vector3> positions)
    {
        Vector3 sum = Vector3.zero;

        foreach (Vector3 position in positions)
            sum += position;

        return sum / positions.Count;
    }

    private float CenterAngle(List<Vector3> positions)
    {
        int width = cloth.numVertexWidth;
        int height = cloth.numVertexHeight;

        int xLeft = Mathf.RoundToInt(0.25f * (width - 1));
        int xMiddle = Mathf.RoundToInt(0.50f * (width - 1));
        int xRight = Mathf.RoundToInt(0.75f * (width - 1));

        int yBottom = Mathf.RoundToInt(0.25f * (height - 1));
        int yMiddle = Mathf.RoundToInt(0.50f * (height - 1));
        int yTop = Mathf.RoundToInt(0.75f * (height - 1));

        Vector3 horizontal =
            positions[GridIndex(xRight, yMiddle)] -
            positions[GridIndex(xLeft, yMiddle)];

        Vector3 vertical =
            positions[GridIndex(xMiddle, yTop)] -
            positions[GridIndex(xMiddle, yBottom)];

        return Vector3.Angle(horizontal, vertical);
    }

    private int GridIndex(int x, int y)
    {
        return x * cloth.numVertexHeight + y;
    }

    private float RmsSpeed(
        List<Vector3> current,
        List<Vector3> previous,
        float sampleDt)
    {
        if (previous == null || previous.Count != current.Count)
            return 0.0f;

        double squaredSpeedSum = 0.0;

        for (int i = 0; i < current.Count; i++)
        {
            Vector3 velocity =
                (current[i] - previous[i]) / sampleDt;

            squaredSpeedSum += velocity.sqrMagnitude;
        }

        return Mathf.Sqrt(
            (float)(squaredSpeedSum / current.Count)
        );
    }

    private float HorizontalDistance(Vector3 a, Vector3 b)
    {
        return Vector2.Distance(
            new Vector2(a.x, a.z),
            new Vector2(b.x, b.z)
        );
    }

    private float LastFiniteValue(List<float> values)
    {
        for (int i = values.Count - 1; i >= 0; i--)
        {
            if (!float.IsNaN(values[i]) &&
                !float.IsInfinity(values[i]))
            {
                return values[i];
            }
        }

        return float.NaN;
    }

    private float MaximumFiniteValue(List<float> values)
    {
        float maximum = float.NaN;

        foreach (float value in values)
        {
            if (float.IsNaN(value) || float.IsInfinity(value))
                continue;

            if (float.IsNaN(maximum) || value > maximum)
                maximum = value;
        }

        return maximum;
    }

    private string CsvNumber(float value)
    {
        if (float.IsNaN(value))
            return "NaN";

        if (float.IsPositiveInfinity(value))
            return "Infinity";

        if (float.IsNegativeInfinity(value))
            return "-Infinity";

        return value.ToString("F6", CultureInfo.InvariantCulture);
    }

    private void OnDestroy()
    {
        if (metricsWriter != null)
        {
            metricsWriter.Flush();
            metricsWriter.Close();
        }

        if (positionsWriter != null)
        {
            positionsWriter.Flush();
            positionsWriter.Close();
        }
    }
}
