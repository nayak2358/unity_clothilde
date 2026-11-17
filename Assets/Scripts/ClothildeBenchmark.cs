using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// Deterministic non-VR benchmarks for Clothilde.
/// Put this file in Assets/Scripts/ClothildeBenchmark.cs.
///
/// EdgeShear:
/// - fixes 3 nodes on the left boundary,
/// - moves 3 corresponding nodes on the right boundary by 0.15 m,
/// - 1 s settle + 2 s smooth motion + 2 s hold.
///
/// TableSlide:
/// - applies no controls,
/// - records free cloth motion for 6 s.
///
/// Results are written to:
/// Assets/Exports/NonVRExperiments/
/// </summary>
public class ClothildeBenchmark : MonoBehaviour
{
    public enum ExperimentMode
    {
        EdgeShear,
        TableSlide
    }

    [Header("Reference")]
    public TriangleMesh cloth;

    [Header("Experiment")]
    public ExperimentMode mode = ExperimentMode.EdgeShear;
    public string runName = "clothilde_run";

    [Header("EdgeShear")]
    public float settleSeconds = 1.0f;
    public float moveSeconds = 2.0f;
    public float holdSeconds = 2.0f;
    public float pullDistance = 0.15f;
    public Vector3 pullDirection = Vector3.forward;

    [Header("TableSlide")]
    public float tableSlideDuration = 6.0f;

    private readonly StringBuilder csv = new StringBuilder();

    private List<Vector3> controlReferencePositions;
    private List<Vector3> previousPositions;

    private Vector3 metricReferenceCenter;
    private float metricReferenceAngle;
    private bool metricReferenceCaptured;

    private float simulationTime;
    private bool saved;

    private float TotalDuration
    {
        get
        {
            if (mode == ExperimentMode.TableSlide)
                return tableSlideDuration;

            return settleSeconds + moveSeconds + holdSeconds;
        }
    }

    private void Start()
    {
        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = 60;

        if (cloth == null)
        {
            Debug.LogError("ClothildeBenchmark: assign the TriangleMesh object.");
            enabled = false;
            return;
        }

        controlReferencePositions = cloth.getTotalMeshPositions();

        if (controlReferencePositions == null ||
            controlReferencePositions.Count == 0)
        {
            Debug.LogError("ClothildeBenchmark: cloth positions are unavailable.");
            enabled = false;
            return;
        }

        previousPositions = new List<Vector3>(controlReferencePositions);

        // For TableSlide the reference configuration is the initial state.
        if (mode == ExperimentMode.TableSlide)
            CaptureMetricReference(controlReferencePositions);

        csv.AppendLine(
            "simulator,experiment,time_s,particle_count,target_offset_m," +
            "com_x,com_y,com_z,com_horizontal_displacement_m," +
            "center_angle_deg,center_angle_change_deg,rms_speed_mps"
        );

        WriteSample(0.0f, controlReferencePositions);
    }

    private void Update()
    {
        if (saved)
            return;

        float nextTime = simulationTime + cloth.dt;

        if (mode == ExperimentMode.EdgeShear)
        {
            ApplyEdgeShear(nextTime);

            // Use the settled state as the deformation reference.
            if (!metricReferenceCaptured && nextTime >= settleSeconds)
                CaptureMetricReference(cloth.getTotalMeshPositions());
        }
    }

    private void LateUpdate()
    {
        if (saved)
            return;

        simulationTime += cloth.dt;

        List<Vector3> currentPositions = cloth.getTotalMeshPositions();

        if (currentPositions == null ||
            currentPositions.Count != controlReferencePositions.Count)
        {
            return;
        }

        WriteSample(simulationTime, currentPositions);
        previousPositions = new List<Vector3>(currentPositions);

        if (simulationTime + 1e-6f >= TotalDuration)
            SaveCsv();
    }

    private void ApplyEdgeShear(float time)
    {
        float offset = CurrentTargetOffset(time);

        Vector3 direction = pullDirection.sqrMagnitude > 0.0f
            ? pullDirection.normalized
            : Vector3.forward;

        int width = cloth.numVertexWidth;
        int height = cloth.numVertexHeight;

        int[] rows = { 0, height / 2, height - 1 };

        foreach (int row in rows)
        {
            int leftIndex = GridIndex(0, row);
            int rightIndex = GridIndex(width - 1, row);

            cloth.DragHandle(
                leftIndex,
                controlReferencePositions[leftIndex]
            );

            cloth.DragHandle(
                rightIndex,
                controlReferencePositions[rightIndex] +
                direction * offset
            );
        }
    }

    private void ReleaseEdgeControls()
    {
        if (mode != ExperimentMode.EdgeShear)
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
        if (mode != ExperimentMode.EdgeShear)
            return 0.0f;

        float t = Mathf.Clamp01(
            (time - settleSeconds) / Mathf.Max(moveSeconds, 1e-6f)
        );

        // Smoothstep.
        t = t * t * (3.0f - 2.0f * t);

        return pullDistance * t;
    }

    private void CaptureMetricReference(List<Vector3> positions)
    {
        metricReferenceCenter = Average(positions);
        metricReferenceAngle = CenterAngle(positions);
        metricReferenceCaptured = true;
    }

    private void WriteSample(
        float time,
        List<Vector3> currentPositions)
    {
        Vector3 centerOfMass = Average(currentPositions);
        float centerAngle = CenterAngle(currentPositions);
        float targetOffset = CurrentTargetOffset(time);

        float horizontalDisplacement = float.NaN;
        float centerAngleChange = float.NaN;

        if (metricReferenceCaptured)
        {
            horizontalDisplacement = HorizontalDistance(
                centerOfMass,
                metricReferenceCenter
            );

            centerAngleChange = Mathf.Abs(
                centerAngle - metricReferenceAngle
            );
        }

        float rmsSpeed = RmsSpeed(
            currentPositions,
            previousPositions,
            Mathf.Max(cloth.dt, 1e-6f)
        );

        csv.AppendLine(string.Format(
            CultureInfo.InvariantCulture,
            "Clothilde,{0},{1:F6},{2},{3:F6}," +
            "{4:F6},{5:F6},{6:F6},{7}," +
            "{8:F6},{9},{10:F6}",
            mode,
            time,
            currentPositions.Count,
            targetOffset,
            centerOfMass.x,
            centerOfMass.y,
            centerOfMass.z,
            CsvNumber(horizontalDisplacement),
            centerAngle,
            CsvNumber(centerAngleChange),
            rmsSpeed
        ));
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
        float dt)
    {
        if (previous == null || previous.Count != current.Count)
            return 0.0f;

        double squaredSpeedSum = 0.0;

        for (int i = 0; i < current.Count; i++)
        {
            Vector3 velocity = (current[i] - previous[i]) / dt;
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

    private string CsvNumber(float value)
    {
        if (float.IsNaN(value))
            return "NaN";

        return value.ToString("F6", CultureInfo.InvariantCulture);
    }

    private void SaveCsv()
    {
        if (saved)
            return;

        saved = true;
        // ReleaseEdgeControls();

        string directory = Path.Combine(
            Application.dataPath,
            "Exports",
            "NonVRExperiments"
        );

        Directory.CreateDirectory(directory);

        string safeRunName = string.IsNullOrWhiteSpace(runName)
            ? "clothilde_" + mode
            : runName.Trim();

        string fileName = string.Format(
            CultureInfo.InvariantCulture,
            "{0}_{1:yyyyMMdd_HHmmss}.csv",
            safeRunName,
            DateTime.Now
        );

        string path = Path.Combine(directory, fileName);

        File.WriteAllText(path, csv.ToString());

        Debug.Log(
            "Clothilde benchmark saved to:\n" + path
        );
    }
}
