using NetMQ;
using NetMQ.Sockets;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.XR;

public class ZmqClientPointCloud : MonoBehaviour
{
    public Material pointCloudMaterial;

    // ============================
    // CONTROLLER TRANSFORM
    // ============================
    public float moveSpeedPerSecond = 0.5f;
    public float rotationSpeedDegreesPerSecond = 60f;
    public float scaleSpeed = 1.5f;

    private Vector3 renderingOffset = Vector3.zero;
    private Quaternion renderingRotation = Quaternion.identity;
    private float renderingScale = 1f;

    // ============================
    // MESH DATA
    // ============================
    private Mesh pointCloudMesh;
    private Vector3[] vertices;
    private Color[] colors;
    private int[] indices;

    private GameObject pcObj;

    // ============================
    // THREAD / ZMQ
    // ============================
    private Thread listenerThread;
    private bool isRunning = false;
    private SubscriberSocket subSocket;

    private short[] sharedxyzData;
    private byte[] sharedrgbData;
    private int sharedPointCount;

    private short[] latestxyzData = null;
    private byte[] latestrgbData = null;
    private int latestPointCount = 0;

    private readonly object Lock = new object();

    void Start()
    {
        pointCloudMesh = new Mesh();
        pointCloudMesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

        vertices = new Vector3[0];
        colors = new Color[0];
        indices = new int[0];

        pcObj = new GameObject("PointCloud");
        MeshFilter mf = pcObj.AddComponent<MeshFilter>();
        MeshRenderer mr = pcObj.AddComponent<MeshRenderer>();
        mf.mesh = pointCloudMesh;
        mr.material = pointCloudMaterial;

        isRunning = true;
        listenerThread = new Thread(ZmqListener);
        listenerThread.IsBackground = true;
        listenerThread.Start();
    }

    // ============================
    // CONTROLLER HANDLING
    // ============================
    void HandleControllers()
    {
        var right = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        var left = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);

        float moveSpeed = moveSpeedPerSecond * Time.deltaTime;
        float rotSpeed = rotationSpeedDegreesPerSecond * Time.deltaTime;

        // Rotation (Right Stick)
        if (right.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 rightStick))
        {
            float yaw = rightStick.x * rotSpeed;
            float pitch = -rightStick.y * rotSpeed;
            renderingRotation *= Quaternion.Euler(pitch, yaw, 0);
        }

        // Translation (Left Stick)
        if (left.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 leftStick))
        {
            renderingOffset += new Vector3(leftStick.x, 0, leftStick.y) * moveSpeed;
        }

        // Up / Down
        if (left.TryGetFeatureValue(CommonUsages.trigger, out float trigger))
            renderingOffset += Vector3.up * trigger * moveSpeed;

        if (left.TryGetFeatureValue(CommonUsages.grip, out float grip))
            renderingOffset += Vector3.down * grip * moveSpeed;

        // Scaling (Buttons)
        if (right.TryGetFeatureValue(CommonUsages.primaryButton, out bool aPressed) && aPressed)
            renderingScale *= 1f + Time.deltaTime * scaleSpeed;

        if (right.TryGetFeatureValue(CommonUsages.secondaryButton, out bool bPressed) && bPressed)
            renderingScale *= 1f - Time.deltaTime * scaleSpeed;
    }

    // ============================
    // ZMQ THREAD
    // ============================
    void ZmqListener()
    {
        AsyncIO.ForceDotNet.Force();

        string serverIp = null;

        while (isRunning && string.IsNullOrEmpty(serverIp))
        {
            serverIp = FindServer();
            if (string.IsNullOrEmpty(serverIp))
                Thread.Sleep(1000);
        }

        using (subSocket = new SubscriberSocket())
        {
            subSocket.Connect($"tcp://{serverIp}:5555");
            subSocket.Subscribe("PointCloud");

            while (isRunning)
            {
                try
                {
                    subSocket.ReceiveFrameString();
                    subSocket.ReceiveFrameBytes(); // ignore length
                    byte[] xyzBytes = subSocket.ReceiveFrameBytes();
                    byte[] rgbBytes = subSocket.ReceiveFrameBytes();

                    int xyzCount = xyzBytes.Length / sizeof(short);
                    if (xyzCount % 3 != 0) continue;

                    int pointCount = xyzCount / 3;
                    if (rgbBytes.Length != pointCount * 3) continue;

                    short[] xyzData = new short[xyzCount];
                    byte[] rgbData = new byte[rgbBytes.Length];

                    Buffer.BlockCopy(xyzBytes, 0, xyzData, 0, xyzBytes.Length);
                    Buffer.BlockCopy(rgbBytes, 0, rgbData, 0, rgbBytes.Length);

                    lock (Lock)
                    {
                        sharedxyzData = xyzData;
                        sharedrgbData = rgbData;
                        sharedPointCount = pointCount;
                    }
                }
                catch (Exception ex)
                {
                    Debug.LogError("[ZMQ] " + ex.Message);
                }
            }
        }
    }

    void ThreadSafeReadingPointCloudData()
    {
        lock (Lock)
        {
            latestxyzData = sharedxyzData;
            latestrgbData = sharedrgbData;
            latestPointCount = sharedPointCount;

            sharedxyzData = null;
            sharedrgbData = null;
        }
    }

    // ============================
    // UPDATE
    // ============================
    void Update()
    {
        HandleControllers();
        ThreadSafeReadingPointCloudData();

        if (latestxyzData == null || latestrgbData == null)
            return;

        if (vertices.Length != latestPointCount)
        {
            vertices = new Vector3[latestPointCount];
            colors = new Color[latestPointCount];
            indices = new int[latestPointCount];

            for (int i = 0; i < latestPointCount; i++)
                indices[i] = i;

            pointCloudMesh.Clear();
            pointCloudMesh.SetIndices(indices, MeshTopology.Points, 0);
        }

        float scale = 0.001f;

        for (int i = 0; i < latestPointCount; i++)
        {
            int idx = i * 3;

            Vector3 localPos = new Vector3(
                latestxyzData[idx] * scale,
                latestxyzData[idx + 1] * scale,
                latestxyzData[idx + 2] * scale
            );

            // Apply transform
            Vector3 transformed =
                renderingRotation * (localPos * renderingScale) + renderingOffset;

            vertices[i] = transformed;

            colors[i] = new Color32(
                latestrgbData[idx],
                latestrgbData[idx + 1],
                latestrgbData[idx + 2],
                255
            );
        }

        pointCloudMesh.SetVertices(vertices);
        pointCloudMesh.SetColors(colors);

        pointCloudMesh.bounds = new Bounds(Vector3.zero, Vector3.one * 100f);

        latestxyzData = null;
        latestrgbData = null;
    }

    // ============================
    // DISCOVERY
    // ============================
    private string FindServer(int timeoutMs = 1000)
    {
        using (UdpClient client = new UdpClient())
        {
            client.EnableBroadcast = true;
            client.Client.ReceiveTimeout = timeoutMs;

            IPEndPoint ep = new IPEndPoint(IPAddress.Broadcast, 5556);
            byte[] req = Encoding.ASCII.GetBytes("DISCOVER_ZMQ_SERVER");
            client.Send(req, req.Length, ep);

            try
            {
                IPEndPoint sender = new IPEndPoint(IPAddress.Any, 0);
                byte[] res = client.Receive(ref sender);

                if (Encoding.ASCII.GetString(res).StartsWith("ZMQ_SERVER_HERE"))
                    return sender.Address.ToString();
            }
            catch { }
        }
        return null;
    }

    // ============================
    // CLEANUP
    // ============================
    void OnDestroy()
    {
        isRunning = false;

        subSocket?.Close();
        subSocket?.Dispose();

        if (listenerThread != null && listenerThread.IsAlive)
            listenerThread.Join(500);

        NetMQConfig.Cleanup();
    }
}