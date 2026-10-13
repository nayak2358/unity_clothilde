using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using Valve.VR;

public class ExporterBin : MonoBehaviour
{
    [Header("Settings")]
    public string targetTag = "Exportable"; // Only objects with this tag will be saved
    private string exportPath = @"C:\Users\maparicio\Documents\Abhilash Nayak\unity_clothilde\Assets\Exports";

    [Header("Scene references")]
    public TriangleMesh cloth;
    public GripperVR leftGripper;
    
    public GripperVR rightGripper;

    private bool isExporting = false;
    private int frame = 0;
    private float recordingStartTime = 0.0f;

    private List<float> timesBin;
    private List<double> clothFramesBin;   // (nFrames, nVertices, 3)
    private List<float> gripperPosesBin;   // (nFrames, 2, 7), order: left, right
    private List<int> jawStatusBin;        // (nFrames, 2), order: left, right

    private double[] meshVerticesBin;      // (nVertices, 3)
    private int[] meshFacesBin;            // (nFaces, 4)

    private int nVertices;
    private int nFaces;

    private const int NGrippers = 2;

    // To export
    public SteamVR_Action_Boolean m_Export = null;
    // Optional: explicitly specify the hand
    public SteamVR_Input_Sources m_HandType = SteamVR_Input_Sources.RightHand;

    void Awake()
    {
        CultureInfo.DefaultThreadCurrentCulture = CultureInfo.InvariantCulture;
    }

    void Update()
    {
        // if (Input.GetKeyDown(KeyCode.P))
        if (m_Export != null && m_Export.GetStateDown(m_HandType))
        {
            if (!isExporting)
                StartRecording();
            else
                StopRecordingAndSave();
        }
    }

    // Record after all normal Update() methods have run, so cloth/gripper poses
    // from the current rendered frame are captured together.
    void LateUpdate()
    {
        if (!isExporting)
            return;

        RecordFrame();
        frame++;
    }

    void StartRecording()
    {
        if (!ValidateReferences())
            return;

        recordingStartTime = Time.time;
        isExporting = true;
        frame = 0;

        Directory.CreateDirectory(exportPath);

        timesBin = new List<float>();
        clothFramesBin = new List<double>();
        gripperPosesBin = new List<float>();
        jawStatusBin = new List<int>();

        ExportMeshOnceBinary();

        Debug.Log("Cloth + left/right gripper export started. Press RightHand/GrabGrip again to stop.");
    }

    bool ValidateReferences()
    {
        bool ok = true;

        if (cloth == null)
        {
            Debug.LogError("Exporter: Cloth (TriangleMesh) reference is missing.");
            ok = false;
        }

        if (leftGripper == null)
        {
            Debug.LogError("Exporter: Left Gripper reference is missing.");
            ok = false;
        }

        if (rightGripper == null)
        {
            Debug.LogError("Exporter: Right Gripper reference is missing.");
            ok = false;
        }

        return ok;
    }

    void StopRecordingAndSave()
    {
        isExporting = false;

        string path = Path.Combine(exportPath, "replay_cloth.bin");
        SaveReplayBinary(path);

        Debug.Log("Export stopped. File saved to: " + path);
    }

    void ExportMeshOnceBinary()
    {
        // TriangleMesh.cs provides getTotalMeshPositions(), which returns
        // the cloth's physical grid vertices in Unity coordinates.
        List<Vector3> verticesUnity = cloth.getTotalMeshPositions();

        nVertices = verticesUnity.Count;
        meshVerticesBin = new double[nVertices * 3];

        for (int i = 0; i < nVertices; i++)
        {
            Vector3 p = UnityPointToPython(verticesUnity[i]);

            meshVerticesBin[3 * i + 0] = p.x;
            meshVerticesBin[3 * i + 1] = p.y;
            meshVerticesBin[3 * i + 2] = p.z;
        }

        int[,] faces = cloth.getFaces();

        nFaces = faces.GetLength(0);
        meshFacesBin = new int[nFaces * 4];

        for (int i = 0; i < nFaces; i++)
        {
            meshFacesBin[4 * i + 0] = faces[i, 0];
            meshFacesBin[4 * i + 1] = faces[i, 1];
            meshFacesBin[4 * i + 2] = faces[i, 2];
            meshFacesBin[4 * i + 3] = faces[i, 3];
        }
    }

    void RecordFrame()
    {
        List<Vector3> verticesUnity = cloth.getTotalMeshPositions();

        if (verticesUnity.Count != nVertices)
        {
            Debug.LogError(
                $"Exporter: cloth vertex count changed during recording " +
                $"({verticesUnity.Count} != {nVertices}). Recording stopped."
            );
            isExporting = false;
            return;
        }

        float t = Time.time - recordingStartTime;
        timesBin.Add(t);

        for (int i = 0; i < nVertices; i++)
        {
            Vector3 p = UnityPointToPython(verticesUnity[i]);
            clothFramesBin.Add(p.x);
            clothFramesBin.Add(p.y);
            clothFramesBin.Add(p.z);
        }

        // Fixed frame-major ordering:
        // frame 0: left pose, right pose
        // frame 1: left pose, right pose
        // ...
        RecordGripper(leftGripper);
        RecordGripper(rightGripper);
    }

    void RecordGripper(GripperVR gripper)
    {
        // In the supplied Gripper.cs, the Gripper object's transform is the
        // grasp-box center / grasp frame. The CAD assembly is offset from it.
        Transform gripperFrame = gripper.graspFrame;

        Vector3 p = UnityPointToPython(gripperFrame.position);
        Vector4 q = UnityQuaternionToPython(gripperFrame.rotation);

        // Pose order: px, py, pz, qw, qx, qy, qz
        gripperPosesBin.Add(p.x);
        gripperPosesBin.Add(p.y);
        gripperPosesBin.Add(p.z);
        gripperPosesBin.Add(q.x); // qw
        gripperPosesBin.Add(q.y); // qx
        gripperPosesBin.Add(q.z); // qy
        gripperPosesBin.Add(q.w); // qz

        // Default to open if the visual assembly is missing.
        int jawOpen = 1;
        if (gripper.gripperAssembly != null)
            jawOpen = gripper.gripperAssembly.IsOpen ? 1 : 0;

        jawStatusBin.Add(jawOpen);
    }

    // Unity point (x, y, z) -> clothilde-sim point (x, z, y - 1)
    Vector3 UnityPointToPython(Vector3 p)
    {
        return new Vector3(p.x, p.z, p.y - 1.0f);
    }

    // Quaternion converted to Python coordinates and returned in the
    // Vector4 fields as (x,y,z,w) = (qw,qx,qy,qz).
    Vector4 UnityQuaternionToPython(Quaternion q)
    {
        return new Vector4(q.w, -q.x, -q.z, -q.y);
    }

    private void SaveReplayBinary(string fileName)
    {
        // Version 2 adds nGrippers to metadata and stores two grippers.
        int version = 2;
        int nFrames = timesBin.Count;
        int nParams = 14;

        float[] simParams = new float[]
        {
            cloth.dt,
            cloth.tol,
            cloth.sub_steps,
            cloth.rho,
            cloth.delta,
            cloth.alpha,
            cloth.kappa,
            cloth.kappa_bnd,
            cloth.str,
            cloth.shr,
            cloth.slf,
            cloth.mu_f,
            cloth.mu_s,
            cloth.thck
        };

        Debug.Log("nFrames = " + nFrames);
        Debug.Log("nVertices = " + nVertices);
        Debug.Log("nFaces = " + nFaces);
        Debug.Log("nGrippers = " + NGrippers);

        using (var fileStream = File.Open(fileName, FileMode.Create))
        using (var writer = new BinaryWriter(fileStream))
        {
            // 8-byte file signature.
            writer.Write(Encoding.ASCII.GetBytes("CLTHSIM1"));

            // Metadata: int32
            writer.Write(version);
            writer.Write(nFrames);
            writer.Write(nVertices);
            writer.Write(nFaces);
            writer.Write(nParams);
            writer.Write(NGrippers);

            // mesh_vertices: float64, shape (nVertices, 3)
            for (int i = 0; i < meshVerticesBin.Length; i++)
                writer.Write(meshVerticesBin[i]);

            // mesh_faces: int32, shape (nFaces, 4)
            for (int i = 0; i < meshFacesBin.Length; i++)
                writer.Write(meshFacesBin[i]);

            // cloth_frames: float64, shape (nFrames, nVertices, 3)
            for (int i = 0; i < clothFramesBin.Count; i++)
                writer.Write(clothFramesBin[i]);

            // gripper_poses: float32, shape (nFrames, 2, 7)
            // gripper index 0 = left, index 1 = right.
            for (int i = 0; i < gripperPosesBin.Count; i++)
                writer.Write(gripperPosesBin[i]);

            // jaw_status: int32, shape (nFrames, 2)
            // gripper index 0 = left, index 1 = right.
            for (int i = 0; i < jawStatusBin.Count; i++)
                writer.Write(jawStatusBin[i]);

            // times: float32, shape (nFrames,)
            for (int i = 0; i < timesBin.Count; i++)
                writer.Write(timesBin[i]);

            // simulator parameters: float32, shape (14,)
            for (int i = 0; i < simParams.Length; i++)
                writer.Write(simParams[i]);
        }
    }
}
