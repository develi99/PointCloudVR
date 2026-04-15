using UnityEngine;
using System;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;

public class ZmqDepthRGBClientComputeShaderRender : MonoBehaviour
{
    public ComputeShader pointCloudCompute;
    public Material pointCloudMaterial;

    public float scaleXY = 0.03f;
    public float maxDepth = 65f;

    private Texture2D rgbTexture;
    private Texture2D depthTexture;

    private Mesh pointCloudMesh;
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

        vertexBuffer = new ComputeBuffer(width * height, sizeof(float) * 3, ComputeBufferType.Structured);
        colorBuffer = new ComputeBuffer(width * height, sizeof(float) * 4, ComputeBufferType.Structured);

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
        if (pointCloudMaterial != null)
        {
            pointCloudMaterial.SetPass(0);
            Graphics.DrawProceduralNow(MeshTopology.Points, width * height);
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
