using System;
using System.Collections.Concurrent;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;
using UnityEngine;
using UnityEngine.Rendering;


public class PointCloudOnQuest : MonoBehaviour
{
    public ComputeShader pointCloudCompute;
    public Material pointCloudMaterial;
    public float maxDepth = 65.0f;
    public float scaleXY = 0.03f;

    private Texture2D rgbTexture;
    private Texture2D depthTexture;

    private Mesh quadMesh;
    private ComputeBuffer vertexBuffer;
    private ComputeBuffer colorBuffer;

    private const int width = 640;
    private const int height = 480;

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
        rgbTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
        depthTexture = new Texture2D(width, height, TextureFormat.R16, false);

        int count = width * height;
        vertexBuffer = new ComputeBuffer(count, sizeof(float) * 3);
        colorBuffer = new ComputeBuffer(count, sizeof(float) * 4);

        GameObject tempQuad = GameObject.CreatePrimitive(PrimitiveType.Quad);
        quadMesh = tempQuad.GetComponent<MeshFilter>().sharedMesh;
        Destroy(tempQuad);

        pointCloudMaterial.SetBuffer("vertexBuffer", vertexBuffer);
        pointCloudMaterial.SetBuffer("colorBuffer", colorBuffer);

        isRunning = true;
        listenerThread = new Thread(ZmqListener);
        listenerThread.IsBackground = true;
        listenerThread.Start();
    }

    void Update()
    {
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

        if (latestRgbBytes != null && latestDepthBytes != null)
        {
            rgbTexture.LoadImage(latestRgbBytes);
            depthTexture.LoadImage(latestDepthBytes);

            DispatchComputeShader();

            latestRgbBytes = null;
            latestDepthBytes = null;


            if (pointCloudMaterial != null && quadMesh != null)
            {
                //pointCloudMaterial.SetBuffer("vertexBuffer", vertexBuffer);
                //pointCloudMaterial.SetBuffer("colorBuffer", colorBuffer);

                pointCloudMaterial.SetPass(0);

                Bounds bounds = new Bounds(Vector3.zero, Vector3.one * 100f);
                Graphics.DrawMeshInstancedProcedural(quadMesh, 0, pointCloudMaterial, bounds, (width * height));
            }
        }
    }

    void DispatchComputeShader()
    {
        int kernel = pointCloudCompute.FindKernel("CSMain");
        pointCloudCompute.SetTexture(kernel, "rgbTex", rgbTexture);
        pointCloudCompute.SetTexture(kernel, "depthTex", depthTexture);
        pointCloudCompute.SetBuffer(kernel, "vertexBuffer", vertexBuffer);
        pointCloudCompute.SetBuffer(kernel, "colorBuffer", colorBuffer);

        pointCloudCompute.SetInt("width", width);
        pointCloudCompute.SetInt("height", height);
        pointCloudCompute.SetFloat("scaleXY", scaleXY);
        pointCloudCompute.SetFloat("maxDepth", maxDepth);

        int threadGroupsX = Mathf.CeilToInt(width / 8.0f);
        int threadGroupsY = Mathf.CeilToInt(height / 8.0f);

        pointCloudCompute.Dispatch(kernel, threadGroupsX, threadGroupsY, 1);
    }
    
    void OnRenderObject()
    {
        Camera cam = Camera.current;
        UnityEngine.Debug.Log(cam);
        if (cam == null)
            return;

        // Optional: nur linkes Auge rendern, um doppelte Ausgabe zu testen
        // if (cam.stereoActiveEye != Camera.MonoOrStereoscopicEye.Left)
        //     return;

        if (pointCloudMaterial != null && quadMesh != null)
        {
            
            pointCloudMaterial.SetPass(0);

            // Bounds möglichst groß, um Sichtfeld abzudecken
            Bounds bounds = new Bounds(cam.transform.position, Vector3.one * 10f);

            //Graphics.DrawMeshInstancedProcedural(quadMesh, 0, pointCloudMaterial, bounds, width * height);
           
        }
    }

    void OnDestroy()
    {
        isRunning = false;

        vertexBuffer?.Release();
        colorBuffer?.Release();

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

                        lock (rgbLock) { sharedRgbBytes = rgbBytes; }
                        lock (depthLock) { sharedDepthBytes = depthBytes; }
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