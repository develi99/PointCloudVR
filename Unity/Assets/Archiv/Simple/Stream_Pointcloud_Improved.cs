using UnityEngine;
using System;
using System.Collections.Concurrent;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;

public class ZmqDepthRGBClientImproved : MonoBehaviour
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

    /* 
     * Here the arrived message from zmq server is saved, be carefull it don't contains 
     * the values, it contains the bytes of jpeg (rgb) and png (depth)
     * Only the actual data is saved, without queue because it isn't needed, only 
     * the actual data ist necessary
     */
    private byte[] sharedRgbBytes; 
    private byte[] sharedDepthBytes;
    private readonly object rgbLock = new object();
    private readonly object depthLock = new object();

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
        // Read latest rgb and depth bytes
        lock (rgbLock)
        {
            latestRgbBytes = sharedRgbBytes;
            sharedRgbBytes = null;
        }
        lock (depthLock)
        {
            latestDepthBytes = sharedDepthBytes;
            sharedDepthBytes = null;
        }

        // If both set, update Point Cloud and wait for new pictures
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
    /*
    void UpdatePointCloud()
    {
        if (rgbTexture == null || depthTexture == null) return;

        float depthRecalculation = 65535f; // The Values are between 0-1, normally they are in mm (0-65635) with that code they are converted to meter
        float depthScaling = 0.001f; // scale down because 1 meter and 1 Coordinate in Visualization is too much
        float XYScaling = 50;
        float maxDepth = 5.0f;    // Optional: maximale Tiefe in Metern

        // Beispielwerte für Kamera-Intrinsiken – bitte anpassen!
        float fx = 887.13f;
        float fy = 887.13f;
        int cx = 640;
        int cy = 359;

        Color[] rgbPixels = rgbTexture.GetPixels();
        Color[] depthPixels = depthTexture.GetPixels();

        int[] count = new int[11];
        for (int i = 0; i < 11; i++) count[i] = 0;

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                /*int flippedY = height - 1 - y;
                int i = y * width + x;
                int flippedI = flippedY * width + x;

                Color depthColor = depthPixels[flippedI];
                float zRaw = depthColor.r; // assumes grayscale depth in R channel
                float z = zRaw * depthScaling;

                // Histogram für Debug
                int bin = Mathf.Clamp((int)(zRaw * 10.0f), 0, 10);
                count[bin]++;

                // Filtere unbrauchbare Tiefenwerte
                if (z <= 0 || z > maxDepth)
                {
                    vertices[i] = Vector3.zero;
                    colors[i] = Color.black;
                    continue;
                }

                // Kamera-Raum Koordinaten
                float X = (x - cx) * z / fx;
                float Y = (y - cy) * z / fy;

                vertices[i] = new Vector3(X, -Y, z);
                colors[i] = rgbPixels[flippedI];
                int i = y * width + x;

                // Kein Flip mehr:
                Color depthColor = depthPixels[i];
                float zRaw = depthColor.r;

                int bin = Mathf.Clamp((int)(zRaw * 10.0f), 0, 10);
                count[bin]++;

                // Filtere unbrauchbare Tiefenwerte
                if (zRaw <= 0 || zRaw > maxDepth)
                {
                    vertices[i] = Vector3.zero;
                    colors[i] = Color.black;
                    continue;
                }

                // Kamera-Raum Koordinaten (kein Y-Flip mehr!)
                float X = (x - cx) * zRaw / fx;
                float Y = (y - cy) * zRaw / fy;

                float z = zRaw * depthScaling;

                X *= XYScaling;
                Y *= XYScaling;

                vertices[i] = new Vector3(X, Y, z); // <- hier kein Minus mehr!
                colors[i] = rgbPixels[i]; // ebenfalls: nicht mehr flippedI verwenden
            }
        }

        // Debug Histogramm
        Debug.Log("\nNew Histogram:");
        for (int i = 0; i < 11; i++) Debug.Log($"{(i / 10.0f):0.0}-{((i + 1.0f) / 10.0f):0.0}: {count[i]}");

        // Punktwolke aktualisieren
        pointCloudMesh.vertices = vertices;
        pointCloudMesh.colors = colors;
        pointCloudMesh.RecalculateBounds();
    }
                */
    void UpdatePointCloud()
    {
        if (rgbTexture == null || depthTexture == null) return;

        float scaleXY = 0.03f;
        float depthScaling = 65f;

        Color[] rgbPixels = rgbTexture.GetPixels();
        Color[] depthPixels = depthTexture.GetPixels();

        int[] count = new int[11];
        for (int i = 0; i < 11; i++) count[i] = 0;

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                //To show the pixels correct, the pixels must be flipped
                int flippedY = height - 1 - y;
                int i = y * width + x;
                int flippedI = flippedY * width + x;

                //Read Depthinformation
                Color depthColor = depthPixels[flippedI];
                float zRaw = depthColor.r; // 0..1 Tiefenwert aus PNG
                                           //float z = Mathf.Pow(zRaw, 0.5f) * depthScaling; // Quadratwurzel streckt nahe Werte
                float z = zRaw * depthScaling;

                count[(int)(zRaw * 10.0)]++;

                //Calculate XYZ from Depth and x,y position in image and add to array
                vertices[i] = new Vector3(
                    (x - width / 2) * scaleXY,
                    (height / 2 - y) * scaleXY, //Try if not good use y
                    z);

                //Add Color to Array
                Color rgbColor = rgbPixels[flippedI];
                colors[i] = rgbColor;
            }
        }
        Debug.Log("\nNew Histogram:");
        for (int i = 0; i < 11; i++) Debug.Log((i / 10.0 * 65535) + "-" + ((i + 1.0) / 10.0 * 65535) + ": " + count[i] + "\n");
        Debug.Log("\n");



        //Add Array, rendering happens automatically
        pointCloudMesh.vertices = vertices;
        pointCloudMesh.colors = colors;
        pointCloudMesh.RecalculateBounds(); // Recalculate Bounding Box
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

        using (subSocket = new PullSocket(">tcp://192.168.0.208:5555"))
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

                        lock (rgbLock) { 
                            sharedRgbBytes = rgbBytes; 
                        }
                        lock (depthLock)
                        {
                            sharedDepthBytes = depthBytes;
                        }
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
