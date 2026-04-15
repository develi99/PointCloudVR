using System;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;
using Newtonsoft.Json.Linq;
using UnityEngine;

public class PointCloudRenderer : MonoBehaviour
{
    public Material pointCloudMaterial;
    public int width = 64;
    public int height = 64;
    public float scaleXY = 0.01f;
    public float pointSize = 10f;

    private ComputeBuffer vertexBuffer;
    private ComputeBuffer colorBuffer;

    void Start()
    {
        int count = width * height;
        vertexBuffer = new ComputeBuffer(count, sizeof(float) * 3);
        colorBuffer = new ComputeBuffer(count, sizeof(float) * 4);

        Vector3[] positions = new Vector3[count];
        Vector4[] colors = new Vector4[count];

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int i = y * width + x;
                positions[i] = new Vector3((x - width / 2) * scaleXY, (y - height / 2) * scaleXY, 2f); // 2m vor Kamera
                colors[i] = new Vector4(x / (float)width, y / (float)height, 1.0f, 1.0f); // Farbe nach Koordinaten
            }
        }

        vertexBuffer.SetData(positions);
        colorBuffer.SetData(colors);

        pointCloudMaterial.SetBuffer("vertexBuffer", vertexBuffer);
        pointCloudMaterial.SetBuffer("colorBuffer", colorBuffer);
        pointCloudMaterial.SetFloat("_PointSize", pointSize);
    }

    void Update()
    {

    }

    void OnRenderObject()
    {
        Debug.Log("Here");
        pointCloudMaterial.SetPass(0);

        // Bounds großzügig wählen
        //Bounds bounds = new Bounds(new Vector3(0, 0, 0), new Vector3(5, 5, 5)*100);
        Graphics.DrawProceduralNow(MeshTopology.Points, width * height);

    }

    void OnDestroy()
    {
        vertexBuffer?.Release();
        colorBuffer?.Release();
    }
}
