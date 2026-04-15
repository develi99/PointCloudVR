using NetMQ;
using NetMQ.Sockets;
using Newtonsoft.Json.Linq;
using System;
using System.Diagnostics;
using System.Threading;
using Unity.VisualScripting;
using UnityEngine;
using static TMPro.SpriteAssetUtilities.TexturePacker_JsonArray;

public class ZmqDepthRGBClientComputeShader : MonoBehaviour
{
    public ComputeShader pointCloudCompute;
    public Material pointCloudMaterial;

    private Texture2D rgbTexture;
    private Mesh pointCloudMesh;
    private ComputeBuffer vertexBuffer;
    private ComputeBuffer depthBuffer; // Neuer Buffer für Tiefendaten
    private Color[] colors;

    private const int width = 640;
    private const int height = 480;

    private byte[] sharedRgbBytes;
    private byte[] sharedDepthBytes;
    private readonly object Lock = new object();

    private Thread listenerThread;
    private bool isRunning = false;
    private PullSocket subSocket;

    private byte[] latestRgbBytes = null;
    private byte[] latestDepthBytes = null;

    bool logPerformance = true;
    private int frames = 0; // FPS
    private float lastTimeFrames = 0.0f;

    void Start()
    {
        rgbTexture = new Texture2D(width, height, TextureFormat.RGB24, false);

        pointCloudMesh = new Mesh();
        pointCloudMesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

        vertexBuffer = new ComputeBuffer(width * height, sizeof(float) * 3);
        depthBuffer = new ComputeBuffer(width * height, sizeof(uint)); // 16 bit passen in uint (alternativ ushort)

        colors = new Color[width * height];

        GameObject pcObj = new GameObject("PointCloud");
        MeshFilter mf = pcObj.AddComponent<MeshFilter>();
        MeshRenderer mr = pcObj.AddComponent<MeshRenderer>();
        mf.mesh = pointCloudMesh;
        mr.material = pointCloudMaterial;

        isRunning = true;
        listenerThread = new Thread(ZmqListener);
        listenerThread.IsBackground = true;
        listenerThread.Start();
    }

    void Update()
    {
        lock (Lock) {
            latestRgbBytes = sharedRgbBytes;
            sharedRgbBytes = null;

            latestDepthBytes = sharedDepthBytes;
            sharedDepthBytes = null;
        }

        if (latestRgbBytes != null && latestDepthBytes != null)
        {
            rgbTexture.LoadImage(latestRgbBytes);

            // Tiefendaten (rohe Bytes) in ushort[] umwandeln
            ushort[] depthUShortArray = new ushort[width * height];
            Buffer.BlockCopy(latestDepthBytes, 0, depthUShortArray, 0, latestDepthBytes.Length);

            // Falls dein Shader uint erwartet, kannst du hier konvertieren
            uint[] depthUintArray = new uint[depthUShortArray.Length];
            for (int i = 0; i < depthUShortArray.Length; i++)
                depthUintArray[i] = depthUShortArray[i];



            //UnityEngine.Debug
            /*
            int[] count = new int[66];
            for (int i = 0; i < 11; i++) count[i] = 0;

            for (int i = 0; i < depthUShortArray.Length; i++)
            {
                count[(int)(depthUShortArray[i] / 1000)]++;
            }

            UnityEngine.Debug.Log("\nNew Histogram:");
            for (int i = 0; i < 65; i++) UnityEngine.Debug.Log((i) + "-" + ((i + 1.0)) + ": " + count[i] + "\n");
            UnityEngine.Debug.Log("\n");
            */


            depthBuffer.SetData(depthUintArray);

            DispatchComputeShader();
            UpdateMeshFromBuffer();

            latestRgbBytes = null;
            latestDepthBytes = null;

            //Calculate FPS
            if (logPerformance)
            {
                frames++;
                float timeNow = Time.realtimeSinceStartup;

                if (timeNow - lastTimeFrames >= 1.0f) // jede Sekunde
                {
                    UnityEngine.Debug.Log($"Real FPS: {frames}");
                    frames = 0;
                    lastTimeFrames = timeNow;
                }
            }
        }
    }

    void DispatchComputeShader()
    {
        int kernel = pointCloudCompute.FindKernel("CSMain");

        pointCloudCompute.SetBuffer(kernel, "DepthBuffer", depthBuffer);
        pointCloudCompute.SetBuffer(kernel, "vertices", vertexBuffer);
        pointCloudCompute.SetInt("width", width);
        pointCloudCompute.SetInt("height", height);

        int threadGroupsX = Mathf.CeilToInt(width / 8.0f);
        int threadGroupsY = Mathf.CeilToInt(height / 8.0f);
        pointCloudCompute.Dispatch(kernel, threadGroupsX, threadGroupsY, 1);
    }

    void UpdateMeshFromBuffer()
    {
        Vector3[] vertexArray = new Vector3[width * height];
        vertexBuffer.GetData(vertexArray);

        Color[] rgbPixels = rgbTexture.GetPixels();
        colors = rgbPixels;

        int[] indices = new int[width * height];
        for (int i = 0; i < indices.Length; i++) indices[i] = i;

        pointCloudMesh.Clear();
        pointCloudMesh.SetVertices(vertexArray);
        pointCloudMesh.SetColors(colors);
        pointCloudMesh.SetIndices(indices, MeshTopology.Points, 0);
        pointCloudMesh.RecalculateBounds();
    }

    void OnDestroy()
    {
        isRunning = false;

        vertexBuffer?.Release();
        depthBuffer?.Release();

        if (subSocket != null)
        {
            try
            {
                subSocket.Close();
                subSocket.Dispose();
            }
            catch (Exception e)
            {
                UnityEngine.Debug.LogWarning("Error closing socket: " + e);
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
                            UnityEngine.Debug.LogWarning("[ZMQ] Nachricht zu kurz für Header");
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
                        
                        lock (Lock) { 
                            sharedRgbBytes = rgbBytes;
                            sharedDepthBytes = depthBytes; 
                        }
                    }
                }
                catch (Exception ex)
                {
                    UnityEngine.Debug.LogError("[ZMQ] Fehler im Listener: " + ex.Message);
                    break;
                }
            }
        }
    }
}
