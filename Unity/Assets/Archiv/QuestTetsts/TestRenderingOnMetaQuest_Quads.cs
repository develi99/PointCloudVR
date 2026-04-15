using UnityEngine;

public class QuadCloudRenderer : MonoBehaviour
{
    public Material pointCloudMaterial;
    public int width = 64;
    public int height = 64;
    public float scaleXY = 0.01f;
    public float maxDepth = 3f;

    ComputeBuffer vertexBuffer;
    ComputeBuffer colorBuffer;
    Mesh quadMesh;

    void Start()
    {
        CreateQuadMesh();
        InitBuffers();
        FillTestData();
    }

    void CreateQuadMesh()
    {
        GameObject tempQuad = GameObject.CreatePrimitive(PrimitiveType.Quad);
        quadMesh = tempQuad.GetComponent<MeshFilter>().sharedMesh;
        Destroy(tempQuad); // Nur das Mesh wird gebraucht
    }

    void InitBuffers()
    {
        int count = width * height;
        vertexBuffer = new ComputeBuffer(count, sizeof(float) * 3);
        colorBuffer = new ComputeBuffer(count, sizeof(float) * 4);

        pointCloudMaterial.SetBuffer("vertexBuffer", vertexBuffer);
        pointCloudMaterial.SetBuffer("colorBuffer", colorBuffer);
    }

    void FillTestData()
    {
        int count = width * height;
        Vector3[] verts = new Vector3[count];
        Vector4[] colors = new Vector4[count];

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int i = y * width + x;
                verts[i] = new Vector3((x - width / 2) * scaleXY, (y - height / 2) * scaleXY, 2f); // 2 Meter vor Kamera
                colors[i] = new Vector4(x / (float)width, y / (float)height, 1, 1);
            }
        }

        vertexBuffer.SetData(verts);
        colorBuffer.SetData(colors);
    }
    void Update()
    {
        if (pointCloudMaterial != null && quadMesh != null)
        {
            pointCloudMaterial.SetPass(0);
            Graphics.DrawMeshInstancedProcedural(quadMesh, 0, pointCloudMaterial, new Bounds(Vector3.zero, Vector3.one * 500f), width * height);
        }
    }

    void OnDestroy()
    {
        vertexBuffer?.Release();
        colorBuffer?.Release();
    }
}
