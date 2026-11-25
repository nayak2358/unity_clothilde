using System;
using System.Globalization;
using System.IO;
using UnityEngine;

// Solver-independent measurements of material-edge strain.
// Only warp and weft edges are included; triangulation diagonals are excluded.
public static class StrainCore
{
    public enum SupportMode
    {
        FullTopEdge,
        TwoTopCorners
    }

    public struct Stats
    {
        public double rms;
        public double max;
        public double above;
        public double rmsU;
        public double rmsV;
    }

    // Material grid indexing: i = u * h + v, with v = 0 at the top.
    public static int[] GetAnchorIndices(int w, int h, SupportMode support)
    {
        if (w < 2 || h < 2)
            throw new ArgumentException("Expected a grid of at least 2 x 2 nodes.");

        if (support == SupportMode.TwoTopCorners)
            return new[] { 0, (w - 1) * h };

        int[] anchors = new int[w];
        for (int u = 0; u < w; u++)
            anchors[u] = u * h;

        return anchors;
    }

    public static Stats Measure(Vector3[] rest, Vector3[] p, int w, int h)
    {
        if (rest.Length != w * h || p.Length != w * h)
            throw new ArgumentException("Grid size mismatch.");

        double sum = 0;
        double su = 0;
        double sv = 0;
        double max = 0;
        int n = 0;
        int nu = 0;
        int nv = 0;
        int above = 0;

        Action<int, int, bool> edge = (a, b, u) =>
        {
            double length = Vector3.Distance(rest[a], rest[b]);
            if (length < 1e-8)
                throw new Exception("Zero rest edge.");

            double e = Vector3.Distance(p[a], p[b]) / length - 1;
            if (double.IsNaN(e) || double.IsInfinity(e))
                throw new Exception("Nonfinite strain.");

            sum += e * e;
            max = Math.Max(max, e);
            n++;

            if (e > 0.01)
                above++;

            if (u)
            {
                su += e * e;
                nu++;
            }
            else
            {
                sv += e * e;
                nv++;
            }
        };

        for (int u = 0; u < w; u++)
        {
            for (int v = 0; v < h; v++)
            {
                int i = u * h + v;
                if (u + 1 < w)
                    edge(i, i + h, true);
                if (v + 1 < h)
                    edge(i, i + 1, false);
            }
        }

        return new Stats
        {
            rms = Math.Sqrt(sum / n),
            max = max,
            above = 100.0 * above / n,
            rmsU = Math.Sqrt(su / nu),
            rmsV = Math.Sqrt(sv / nv)
        };
    }

    public static string F(double d)
    {
        return d.ToString("G10", CultureInfo.InvariantCulture);
    }

    public static string HUD(Stats s, int substeps, float fps)
    {
        return string.Format(
            CultureInfo.InvariantCulture,
            "RMS strain: {0:F3}%\nMax tensile: {1:F3}%\n" +
            "Edges >1%: {2:F2}%\nSubsteps: {3}\nFPS: {4:F1}",
            100 * s.rms,
            100 * s.max,
            s.above,
            substeps,
            fps
        );
    }

    public static string Folder(string sim, string run)
    {
        foreach (char c in Path.GetInvalidFileNameChars())
            run = run.Replace(c, '_');

        string path = Path.Combine(
            Application.dataPath,
            "Exports",
            "StaticStretch",
            sim + "_" + run + "_" + DateTime.Now.ToString("yyyyMMdd_HHmmss_fff")
        );

        Directory.CreateDirectory(path);
        return path;
    }

    public class Log : IDisposable
    {
        private readonly StreamWriter writer;
        private readonly int w;
        private readonly int h;
        private readonly float dt;
        private readonly int[] anchors;
        private Vector3[] prev;
        private int step;
        private bool disposed;

        public readonly string folder;
        public readonly Vector3[] rest;

        public Log(
            string sim,
            string run,
            int substeps,
            double compliance,
            int iterations,
            float dt,
            Vector3[] rest,
            int w,
            int h,
            string extra,
            float duration = 5f,
            SupportMode support = SupportMode.FullTopEdge)
        {
            this.rest = rest;
            this.w = w;
            this.h = h;
            this.dt = dt;
            anchors = GetAnchorIndices(w, h, support);
            prev = (Vector3[])rest.Clone();
            folder = Folder(sim, run);

            File.WriteAllText(
                Path.Combine(folder, "settings.csv"),
                "simulator,substeps,compliance,iterations,dt,width,height," +
                "duration_s,support_mode,anchor_count\n" +
                sim + "," + substeps + "," + F(compliance) + "," +
                iterations + "," + F(dt) + "," + w + "," + h + "," +
                F(duration) + "," + support + "," + anchors.Length + "\n"
            );

            File.WriteAllText(Path.Combine(folder, "notes.txt"), extra);
            writer = new StreamWriter(Path.Combine(folder, "metrics.csv"));
            writer.WriteLine(
                "step,time_s,solver_ms,rms_strain,max_tensile,pct_gt_1," +
                "rms_u,rms_v,rms_speed_mps,anchor_max_error_m"
            );

            using (var f = new StreamWriter(Path.Combine(folder, "rest.csv")))
            {
                f.WriteLine("node,x,y,z");
                for (int i = 0; i < rest.Length; i++)
                {
                    f.WriteLine(
                        i + "," + F(rest[i].x) + "," +
                        F(rest[i].y) + "," + F(rest[i].z)
                    );
                }
            }

            // Save the actual fixed material-node IDs for reproducibility.
            using (var f = new StreamWriter(Path.Combine(folder, "anchors.csv")))
            {
                f.WriteLine("node,x,y,z");
                foreach (int i in anchors)
                {
                    f.WriteLine(
                        i + "," + F(rest[i].x) + "," +
                        F(rest[i].y) + "," + F(rest[i].z)
                    );
                }
            }
        }

        public void Sample(Vector3[] p, double ms)
        {
            step++;
            Stats s = Measure(rest, p, w, h);
            double speed = 0;
            double anchorError = 0;

            for (int i = 0; i < p.Length; i++)
                speed += (p[i] - prev[i]).sqrMagnitude;

            // Only the nodes that are actually fixed should be checked.
            foreach (int i in anchors)
                anchorError = Math.Max(anchorError, Vector3.Distance(rest[i], p[i]));

            writer.WriteLine(string.Join(
                ",",
                new[]
                {
                    step.ToString(),
                    F(step * dt),
                    F(ms),
                    F(s.rms),
                    F(s.max),
                    F(s.above),
                    F(s.rmsU),
                    F(s.rmsV),
                    F(Math.Sqrt(speed / p.Length) / dt),
                    F(anchorError)
                }
            ));

            prev = (Vector3[])p.Clone();
            if (step % 30 == 0)
                writer.Flush();
        }

        public void Dispose()
        {
            if (disposed)
                return;

            disposed = true;
            writer.Dispose();
            Debug.Log("Saved " + folder);
        }
    }
}
