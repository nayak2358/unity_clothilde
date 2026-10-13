using System.Collections.Generic;
using UnityEngine;

public class Table : MonoBehaviour
{
    [SerializeField] private string targetName = "MeshRenderer";
    public Vector3 offset = new Vector3(1.2f, 0.9f, 0f);
    public float mu = 0.3f;
    public Vector3 dimensions = new Vector3(0.6f, 0.05f, 0.6f);
    public bool generateTable = false;

    void Start()
    {
        PositionTable();
    }

    public void PositionTable()
    {
        transform.position = Vector3.zero;
        transform.localScale = dimensions;

        GameObject targetObj = GameObject.Find(targetName);

        TriangleMesh mesh = targetObj.GetComponent<TriangleMesh>();
        Renderer targetRenderer = targetObj.GetComponent<Renderer>();
        Renderer tableRenderer = GetComponent<Renderer>();
        float tableHalfHeight = 0f;

        if (tableRenderer != null)
        {
            tableHalfHeight = tableRenderer.bounds.extents.y;
        }


        float meshTopY = mesh.transform.position.y;

        // Calculate center Y position so the top of the table sits just below the cloth
        // float tableCenterY = meshTopY - dimensions[2]/2 - 0.005f;

        // Vector3 newPosition = new Vector3(
        //     targetObj.transform.position.x + mesh.gridWidth/2,
        //     tableCenterY,           
        //     targetObj.transform.position.z + mesh.gridHeight/2
        // );

        Vector3 newPosition = offset;

        transform.position = newPosition;

        Debug.Log($"[Table] Successfully placed {gameObject.name} above {targetName}.");
    }
}