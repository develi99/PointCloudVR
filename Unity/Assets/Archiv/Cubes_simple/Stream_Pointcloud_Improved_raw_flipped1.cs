using NetMQ;
using NetMQ.Sockets;
using System;
using System.CodeDom;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.XR;


public class InstancedCubeRendererSimple : MonoBehaviour
{
    public float CubeSize = 0.05f;
    public float scale = 50.0f;

    private Texture2D rgbTexture;
    private uint[] depth;


    //These Parameters are used in Start for init. The real size is set dynamically at runtime
    private int g_width = 640;
    private int g_height = 480;


    //Networkstuff and related Buffers
    private byte[] latestRgbBytes = null;
    private byte[] latestDepthBytes = null;
    private byte[] sharedRgbBytes;
    private byte[] sharedDepthBytes;
    private readonly object Lock = new object();
    private byte[] lastMsg;

    private Thread listenerThread;
    private bool isRunning = false;
    private PullSocket subSocket;


    //FPS Log
    public bool logPerformance = true;
    private int frames = 0;
    private float lastTimeFrames = 0.0f;


    //Rendering Stuff
    public Material instancedMaterial;

    private Mesh cubeMesh;
    private Matrix4x4[] matrices;

    private ComputeBuffer matrixBuffer;
    private ComputeBuffer argsBuffer;
    private ComputeBuffer colorBuffer;
    private Bounds renderBounds;


    //Camera intrinsics
    float fx;
    float fy;
    float cx;
    float cy;


    void Start()
    {
        fx = 591.4252f;
        fy = 591.4252f;
        cx = 320.1326f;
        cy = 239.1477f;

        GameObject temp = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cubeMesh = temp.GetComponent<MeshFilter>().sharedMesh;
        Destroy(temp);

        //This buffer do not change while runtime, it's always the same size
        argsBuffer = new ComputeBuffer(1, 5 * sizeof(uint), ComputeBufferType.IndirectArguments);
        renderBounds = new Bounds(Vector3.zero, Vector3.one * 100);

        //Init changable Buffers, depending from width and height
        InitBuffers(g_width, g_height);

        //Start network thread
        isRunning = true;
        listenerThread = new Thread(ZmqListener);
        listenerThread.IsBackground = true;
        listenerThread.Start();
    }


    /*
     * here all buffers that could change while runtime are placed
     */
    void InitBuffers(int l_width, int l_height)
    {
        rgbTexture = new Texture2D(l_width, l_height, TextureFormat.RGB24, false);
        depth = new uint[l_width * l_height];

        matrices = new Matrix4x4[l_width * l_height];
        matrixBuffer = new ComputeBuffer(l_width * l_height, Marshal.SizeOf(typeof(Matrix4x4)));
        instancedMaterial.SetBuffer("matrixBuffer", matrixBuffer);

        instancedMaterial.SetTexture("_ColorTex", rgbTexture);
        instancedMaterial.SetInt("_Width", l_width);

        uint[] args = new uint[5] {
            (cubeMesh != null) ? cubeMesh.GetIndexCount(0) : 0,
            (uint) (l_width * l_height),
            (cubeMesh != null) ? cubeMesh.GetIndexStart(0) : 0,
            (cubeMesh != null) ? cubeMesh.GetBaseVertex(0) : 0,
            0
        };
        argsBuffer.SetData(args);
    }


    void Update()
    {
        int l_width;
        int l_height;

        lock (Lock)
        {
            latestDepthBytes = sharedDepthBytes;
            latestRgbBytes = sharedRgbBytes;
            sharedRgbBytes = null;
            sharedDepthBytes = null;
            l_width = g_width;
            l_height = g_height;
        }

        //if new data is available
        if (latestDepthBytes != null && latestRgbBytes != null)
        {

            if (rgbTexture.width != l_width || rgbTexture.height != l_height)
                InitBuffers(l_width, l_height);

            rgbTexture.LoadImage(latestRgbBytes);
            setDepthBuffer();

            ComputeMatrices(l_width, l_height);

            latestDepthBytes = null;
            latestRgbBytes = null;
        }

        Graphics.DrawMeshInstancedIndirect(
             cubeMesh,
             0,
             instancedMaterial,
             renderBounds,
             argsBuffer
        );

        if (logPerformance)
            logFPS();
    }


    void logFPS()
    {

        frames++;
        float timeNow = Time.realtimeSinceStartup;

        if (timeNow - lastTimeFrames >= 1.0f)
        {
            UnityEngine.Debug.Log($"Real FPS: {frames}");
            frames = 0;
            lastTimeFrames = timeNow;
        }
    }


    void setDepthBuffer()
    {
        //Read the bytes as UShort
        ushort[] depthUshorts = new ushort[latestDepthBytes.Length / 2];
        Buffer.BlockCopy(latestDepthBytes, 0, depthUshorts, 0, latestDepthBytes.Length);

        //To ints
        for (int i = 0; i < depthUshorts.Length; i++)
        {
            depth[i] = depthUshorts[i];
        }
    }


    void ComputeMatrices(int l_width, int l_height)
    {
        for (int y = 0; y < l_height; y++)
        {
            for (int x = 0; x < l_width; x++)
            {
                // Flip Y wie im Shader
                int flippedY = l_height - 1 - y;
                int index = flippedY * l_width + x;

                uint depthRaw = depth[index];
                float z = depthRaw * 0.001f; // Tiefenwerte in Meter

                // Berechnung der 3D Position
                float X = (x - cx) * z / fx;
                float Y = (y - cy) * z / fy;

                // Matrix direkt zusammenbauen wie im Shader (Zeilenweise)
                // Unity: Matrix4x4 nimmt Spaltenvektoren, also bauen wir die Matrix entsprechend
                Matrix4x4 mat = new Matrix4x4();

                // Erste Zeile (x-Achse + Translation X)
                mat.SetRow(0, new Vector4(CubeSize * scale, 0, 0, X * scale));
                // Zweite Zeile (y-Achse + Translation Y)
                mat.SetRow(1, new Vector4(0, CubeSize * scale, 0, Y * scale));
                // Dritte Zeile (z-Achse + Translation Z)
                mat.SetRow(2, new Vector4(0, 0, CubeSize * scale, z * scale));
                // Vierte Zeile (Homogene Koordinaten)
                mat.SetRow(3, new Vector4(0, 0, 0, 1));

                matrices[l_width * y + x] = mat;
            }
        }

        matrixBuffer.SetData(matrices);
    }


    void OnDestroy()
    {
        matrixBuffer?.Release();
        argsBuffer?.Release();
        colorBuffer?.Release();

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
                UnityEngine.Debug.LogWarning("Error closing socket: " + e);
            }
        }

        if (listenerThread != null && listenerThread.IsAlive)
        {
            listenerThread.Join(500);
        }

        NetMQConfig.Cleanup();
    }


    private string FindServer(int timeoutMs = 1000)
    {
        string discoveredIp = null;
        UdpClient client = new UdpClient();
        client.EnableBroadcast = true;
        client.Client.ReceiveTimeout = timeoutMs;

        IPEndPoint broadcastEp = new IPEndPoint(IPAddress.Broadcast, 5556);
        byte[] request = Encoding.ASCII.GetBytes("DISCOVER_ZMQ_SERVER");
        client.Send(request, request.Length, broadcastEp);

        try
        {
            IPEndPoint senderEp = new IPEndPoint(IPAddress.Any, 0);
            byte[] response = client.Receive(ref senderEp);
            string msg = Encoding.ASCII.GetString(response);

            if (msg.StartsWith("ZMQ_SERVER_HERE"))
            {
                discoveredIp = senderEp.Address.ToString();
                UnityEngine.Debug.Log("[ZMQ] Server found: " + discoveredIp);
            }
        }
        catch (SocketException)
        { }

        client.Close();
        return discoveredIp;
    }


    void ZmqListener()
    {
        AsyncIO.ForceDotNet.Force();

        string serverIp = null;
        bool serverfound = false;
        while (!serverfound)
        {

            serverIp = FindServer();
            if (string.IsNullOrEmpty(serverIp))
            {
                UnityEngine.Debug.LogError("[ZMQ] No Server found.");
                System.Threading.Thread.Sleep(1000); //1 sec
                continue;
            }

            serverfound = true;
        }

        using (subSocket = new PullSocket($">tcp://{serverIp}:5555"))
        {
            while (isRunning)
            {
                try
                {
                    //Only process the latest data
                    lastMsg = null;
                    while (subSocket.TryReceiveFrameBytes(TimeSpan.Zero, out byte[] msg))
                        lastMsg = msg;

                    if (lastMsg == null)
                        continue;

                    if (lastMsg.Length < 8)
                    {
                        UnityEngine.Debug.LogWarning("[ZMQ] Message to short");
                        continue;
                    }
                    int offset = 0;

                    // 1. Size (Width & Height)
                    if (lastMsg.Length < offset + 8) return;
                    int l_width = BitConverter.ToInt32(lastMsg, offset); offset += 4;
                    int l_height = BitConverter.ToInt32(lastMsg, offset); offset += 4;

                    // 2. RGB-Length
                    if (lastMsg.Length < offset + 4) return;
                    int rgbLen = BitConverter.ToInt32(lastMsg, offset); offset += 4;

                    // 3. RGB-Data
                    if (lastMsg.Length < offset + rgbLen) return;
                    byte[] rgbBytes = new byte[rgbLen];
                    Buffer.BlockCopy(lastMsg, offset, rgbBytes, 0, rgbLen);
                    offset += rgbLen;

                    // 4. Depth-Length
                    if (lastMsg.Length < offset + 4) return;
                    int depthLen = BitConverter.ToInt32(lastMsg, offset); offset += 4;

                    // 5. Depth-Data
                    if (lastMsg.Length < offset + depthLen) return;
                    byte[] depthBytes = new byte[depthLen];
                    Buffer.BlockCopy(lastMsg, offset, depthBytes, 0, depthLen);

                    // Safe
                    lock (Lock)
                    {
                        sharedRgbBytes = rgbBytes;
                        sharedDepthBytes = depthBytes;
                        this.g_width = l_width;
                        this.g_height = l_height;
                    }
                }
                catch (Exception ex)
                {
                    UnityEngine.Debug.LogError("[ZMQ] Fault in Listener: " + ex.Message);
                    break;
                }
            }
        }
    }
}
