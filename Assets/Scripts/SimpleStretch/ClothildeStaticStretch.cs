using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

/// <summary>
/// Vertical static-stretch benchmark for Clothilde's KeyboardScene branch.
/// Uses the existing Python simulation, positional controls and timing list.
/// </summary>
[DefaultExecutionOrder(-1000)]
public class ClothildeStaticStretch : MonoBehaviour
{
    [Header("Reference")]
    public TriangleMesh cloth;

    [Header("Experiment")]
    public string runName = "trial1";
    public float duration = 5f;
    public StrainCore.SupportMode supportMode = StrainCore.SupportMode.FullTopEdge;

    private StrainCore.Log log;
    private List<float> existingTimes;
    private int count;
    private int steps;
    private int lastCount;
    private bool finished;

    private void Start()
    {
        if (!cloth || !cloth.enabled)
            throw new Exception("Assign an enabled TriangleMesh.");

        if (cloth.numVertexWidth != 21 || cloth.numVertexHeight != 21)
            throw new Exception("Use a 21 x 21 grid.");

        if (cloth.dt <= 0 || duration < cloth.dt)
            throw new Exception("Invalid timestep or duration.");

        // Read the undeformed physical grid, not the visualization vertices.
        Vector3[] rest = cloth.getTotalMeshPositions().ToArray();
        if (Mathf.Abs(rest[0].y - rest[20].y - 0.7f) > 1e-4f ||
            Mathf.Abs(Vector3.Distance(rest[0], rest[420]) - 0.7f) > 1e-4f ||
            Mathf.Abs(rest[0].z - rest[20].z) > 1e-4f)
        {
            throw new Exception("Expected a vertical 0.7 m cloth, top v = 0.");
        }

        // Reuse the existing private profiling list; no core patch or new timer.
        FieldInfo field = typeof(TriangleMesh).GetField(
            "simulateTime",
            BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public
        );
        existingTimes = field?.GetValue(cloth) as List<float>;
        if (existingTimes == null)
            throw new Exception("Existing TriangleMesh.simulateTime list not found.");

        lastCount = existingTimes.Count;
        steps = Mathf.RoundToInt(duration / cloth.dt);

        log = new StrainCore.Log(
            "Clothilde",
            runName,
            cloth.sub_steps,
            cloth.str,
            -1,
            cloth.dt,
            rest,
            21,
            21,
            "Existing Python lastTime, seconds converted to ms; rho=" + cloth.rho +
            "; tol=" + cloth.tol + "; shr=" + cloth.shr,
            duration,
            supportMode
        );

        // Fix the chosen material nodes before the first simulation Update.
        int[] anchors = StrainCore.GetAnchorIndices(21, 21, supportMode);
        foreach (int i in anchors)
            cloth.DragHandle(i, rest[i]);
    }

    private void LateUpdate()
    {
        if (log == null || finished)
            return;

        // TriangleMesh must have simulated exactly one frame since the last sample.
        if (existingTimes.Count != lastCount + 1)
        {
            Debug.LogError(
                "Expected one existing Python profiling sample per frame. " +
                "Do not toggle profiling (T). Aborting incomplete run."
            );
            Finish();
            return;
        }

        lastCount = existingTimes.Count;
        log.Sample(
            cloth.getTotalMeshPositions().ToArray(),
            1000.0 * existingTimes[lastCount - 1]
        );

        if (++count >= steps)
            Finish();
    }

    private void Finish()
    {
        finished = true;
        log?.Dispose();
        log = null;
        cloth.enabled = false;
    }

    private void OnDestroy()
    {
        log?.Dispose();
        log = null;
    }
}
