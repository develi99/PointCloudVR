using UnityEngine;
using System;
using System.Collections.Concurrent;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;

public class ZmqDepthRGBClient : MonoBehaviour
{
    public Material pointCloudMaterial;

    private Texture2D rgbTexture;
    private Texture2D depthTexture;

    private Mesh pointCloudMesh;
    private Vector3[] vertices;
    private Color[] colors;
    private int[] indices;

    private const int width = 640;
    private const int height = 480;

    private ConcurrentQueue<byte[]> rgbQueue = new ConcurrentQueue<byte[]>();
    private ConcurrentQueue<byte[]> depthQueue = new ConcurrentQueue<byte[]>();

    private Thread listenerThread;
    private bool isRunning = false;
    private PullSocket subSocket;

    private byte[] latestRgbBytes = null;
    private byte[] latestDepthBytes = null;

    void Start()
    {
        // Create to rgb textures without mipmapping (false)
        rgbTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
        depthTexture = new Texture2D(width, height, TextureFormat.RGB24, false);

        // Create Mesh for the Pointclouds with respective Arrays
        pointCloudMesh = new Mesh();
        pointCloudMesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

        vertices = new Vector3[width * height];
        colors = new Color[width * height];
        indices = new int[width * height];

        //Fill Indices
        for (int i = 0; i < indices.Length; i++)
            indices[i] = i;

        pointCloudMesh.vertices = vertices;
        pointCloudMesh.colors = colors;
        pointCloudMesh.SetIndices(indices, MeshTopology.Points, 0);

        //Create Empty Game Object
        GameObject pcObj = new GameObject("PointCloud");
        MeshFilter mf = pcObj.AddComponent<MeshFilter>();
        MeshRenderer mr = pcObj.AddComponent<MeshRenderer>();
        mf.mesh = pointCloudMesh;
        mr.material = pointCloudMaterial; // Material is public and set from outside

        //Start Network Thread
        isRunning = true;
        listenerThread = new Thread(ZmqListener);
        listenerThread.IsBackground = true;
        listenerThread.Start();
    }

    void Update()
    {
        // Read all Bytes in Queue
        while (rgbQueue.TryDequeue(out byte[] rgbBytes))
        {
            latestRgbBytes = rgbBytes;
        }
        while (depthQueue.TryDequeue(out byte[] depthBytes))
        {
            latestDepthBytes = depthBytes;
        }

        if (latestRgbBytes != null && latestDepthBytes != null)
        {
            rgbTexture.LoadImage(latestRgbBytes);
            depthTexture.LoadImage(latestDepthBytes);

            UpdatePointCloud();

            // Nach Verarbeitung resetten
            latestRgbBytes = null;
            latestDepthBytes = null;
        }
    }

    void UpdatePointCloud()
    {
        if (rgbTexture == null || depthTexture == null) return;

        float scaleXY = 0.03f;
        float maxDepthMeters = 5f;

        Color[] rgbPixels = rgbTexture.GetPixels();
        Color[] depthPixels = depthTexture.GetPixels();

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {

                int flippedY = height - 1 - y;
                int i = y * width + x;
                int flippedI = flippedY * width + x;


                //int i = y * width + x;

                //Color depthColor = depthTexture.GetPixel(x, y);
                Color depthColor = depthPixels[flippedI];
                float depthValue = depthColor.r;

                float zRaw = depthColor.r; // 0..1 Tiefenwert aus PNG
                float z = Mathf.Pow(zRaw, 0.5f) * maxDepthMeters; // Quadratwurzel streckt nahe Werte

                //float z = depthValue * maxDepthMeters; lineare Darstellung

                vertices[i] = new Vector3(
                    (x - width / 2) * scaleXY,
                    (height / 2 - flippedY) * scaleXY,
                    z);

                //Color rgbColor = rgbTexture.GetPixel(x, y);
                Color rgbColor = rgbPixels[flippedI];
                colors[i] = rgbColor;
            }
        }

        pointCloudMesh.vertices = vertices;
        pointCloudMesh.colors = colors;
        pointCloudMesh.RecalculateBounds();
    }

    void OnDestroy()
    {
        isRunning = false;

        if (subSocket != null)
        {
            try
            {
                subSocket.Close();
                subSocket.Dispose();
            }
            catch (Exception e)
            {
                Debug.LogWarning("Error closing socket: " + e);
            }
        }

        if (listenerThread != null && listenerThread.IsAlive)
        {
            listenerThread.Join(500);
        }

        NetMQConfig.Cleanup();
    }

    void ZmqListener()
    {
        AsyncIO.ForceDotNet.Force();

        using (subSocket = new PullSocket(">tcp://localhost:5555"))
        {
            while (isRunning)
            {
                try
                {
                    if (subSocket.TryReceiveFrameBytes(TimeSpan.FromMilliseconds(100), out byte[] msg))
                    {
                        if (msg.Length < 8)
                        {
                            Debug.LogWarning("[ZMQ] Nachricht zu kurz für Header");
                            continue;
                        }

                        int rgbLen = BitConverter.ToInt32(msg, 0);
                        if (msg.Length < 4 + rgbLen + 4) continue;

                        byte[] rgbBytes = new byte[rgbLen];
                        Buffer.BlockCopy(msg, 4, rgbBytes, 0, rgbLen);

                        int depthLen = BitConverter.ToInt32(msg, 4 + rgbLen);
                        if (msg.Length < 4 + rgbLen + 4 + depthLen) continue;

                        byte[] depthBytes = new byte[depthLen];
                        Buffer.BlockCopy(msg, 4 + rgbLen + 4, depthBytes, 0, depthLen);

                        rgbQueue.Enqueue(rgbBytes);
                        depthQueue.Enqueue(depthBytes);
                    }
                }
                catch (Exception ex)
                {
                    Debug.LogError("[ZMQ] Fehler im Listener: " + ex.Message);
                    break;
                }
            }
        }
    }
}
