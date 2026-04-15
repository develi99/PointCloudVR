using UnityEngine;
using System;
using System.Collections.Concurrent;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;

public class ZmqDepthRGBClientImprovedRaw : MonoBehaviour
{
    public Material pointCloudMaterial;

    private Texture2D rgbTexture;
    private ushort[] depthUShortArray;

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
        depthUShortArray = new ushort[width * height];

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
    int count = 0;
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
            // Tiefendaten (rohe Bytes) in ushort[] umwandeln
            Buffer.BlockCopy(latestDepthBytes, 0, depthUShortArray, 0, latestDepthBytes.Length);

            Debug.Log("Update" + count++);
            /*
            int[] count = new int[66];
            for (int i = 0; i < 11; i++) count[i] = 0;

            for (int i = 0; i < depthUShortArray.Length; i++)
            {
                count[(int)(depthUShortArray[i] / 1000)]++;
            }

            Debug.Log("\nNew Histogram:");
            for (int i = 0; i < 65; i++) Debug.Log((i) + "-" + ((i + 1.0)) + ": " + count[i] + "\n");
            Debug.Log("\n");
            */

            UpdatePointCloud();

            // Nach Verarbeitung resetten
            latestRgbBytes = null;
            latestDepthBytes = null;
        }
    }

    void UpdatePointCloud()
    {

        float toMeter = 0.001f; // The Values are between 0-1, normally they are in mm (0-65635) with that code they are converted to meter
        float scale = 50f;
        float maxDepth = 5.0f;    // Optional: maximale Tiefe in Metern

        /* 1280x720
        float fx = 887.1378784179688f;
        float fy = 887.1378784179688f;
        float cx = 640.1989135742188f;
        float cy = 358.7215270996094f;
        */

        float fx = 591.4252319335938f;
        float fy = 591.4252319335938f;
        float cx = 320.1325988769531f;
        float cy = 239.1476745605468f;


        Color[] rgbPixels = rgbTexture.GetPixels();

        int[] count = new int[11];
        for (int i = 0; i < 11; i++) count[i] = 0;

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int i = y * width + x;

                // Kein Flip mehr:
                float z = depthUShortArray[i] * toMeter;

                // Filtere unbrauchbare Tiefenwerte
                if (z <= 0 || z > maxDepth)
                {
                    vertices[i] = Vector3.zero;
                    colors[i] = Color.black;
                    continue;
                }

                // Kamera-Raum Koordinaten (kein Y-Flip mehr!)
                float X = (x - cx) * z / fx;
                float Y = (y - cy) * z / fy;

                vertices[i] = new Vector3(X * scale, Y * scale, z*scale);
                colors[i] = rgbPixels[i]; // ebenfalls: nicht mehr flippedI verwenden
            }
        }

        Debug.Log("Update Point Cloud" + count);

        // Punktwolke aktualisieren
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
