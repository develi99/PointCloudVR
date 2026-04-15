using NetMQ;
using NetMQ.Sockets;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.XR;
using TMPro;


public class ZmqClientPointCloudParticles : MonoBehaviour
{
    public TextMeshProUGUI text;
    float timeAccumulator = 0f;
    int frameCount = 0;

    public float particleSize = 0.01f;
    public Material particleMaterial;

    // Controller Settings
    public float moveSpeedPerSecond = 0.5f;
    public float rotationSpeedDegreesPerSecond = 60f;
    public float scaleSpeed = 1.5f;

    private Vector3 renderingOffset = Vector3.zero;
    private Quaternion renderingRotation = Quaternion.identity;
    private float renderingScale = 1f;

    private ParticleSystem ps;
    private ParticleSystem.Particle[] particles;

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
        // Particle System
        ps = gameObject.AddComponent<ParticleSystem>();

        var renderer = ps.GetComponent<ParticleSystemRenderer>();
        var shader = Shader.Find("Unlit/Color");
        if(shader == null)
            UnityEngine.Debug.Log("[OwnMessage] Shader not found");
        //Material mat = new Material(Shader.Find("Particles/Standard Unlit"));
        //Material mat = new Material(Shader.Find("Universal Render Pipeline/Particles/Unlit"));
        Material mat = particleMaterial;



        renderer.material = mat;
        renderer.renderMode = ParticleSystemRenderMode.Billboard;

        var main = ps.main;
        main.loop = false;
        main.playOnAwake = false;
        main.maxParticles = 1000000;
        main.startLifetime = float.MaxValue;
        main.startSpeed = 0;
        main.simulationSpace = ParticleSystemSimulationSpace.World;
        main.startColor = Color.white;

        ps.Play();

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

        float rot;

        // =========================
        // RIGHT STICK
        // X -> YAW (Y AXIS)
        // Y -> PITCH (X AXIS)
        // =========================
        if (right.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 rightStick))
        {
            rot = rightStick.x * rotSpeed;
            renderingRotation *= Quaternion.Euler(0f, rot, 0f); // Y axis (yaw)
        }

        // =========================
        // TRIGGER -> Z AXIS (+)
        // =========================
        if (right.TryGetFeatureValue(CommonUsages.trigger, out float trigger))
        {
            renderingRotation *= Quaternion.Euler(-trigger * rotSpeed, 0f, 0f);
        }

        // =========================
        // GRIP -> Z AXIS (-)
        // =========================
        if (right.TryGetFeatureValue(CommonUsages.grip, out float grip))
        {
            renderingRotation *= Quaternion.Euler(grip * rotSpeed, 0f, 0f);
        }

        // -------- Right X / Y --------
        if (right.TryGetFeatureValue(CommonUsages.primaryButton, out bool primary) && primary)
        {
            renderingRotation *= Quaternion.Euler(0f, 0f, -rotSpeed); // extra X
        }

        if (right.TryGetFeatureValue(CommonUsages.secondaryButton, out bool secondary) && secondary)
        {
            renderingRotation *= Quaternion.Euler(0f, 0f, rotSpeed); // extra Y
        }



        // =========================
        // LEFT STICK: TRANSLATION XZ
        // =========================
        if (left.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 leftStick))
        {
            renderingOffset += new Vector3(leftStick.x, 0f, leftStick.y) * moveSpeed;
        }

        // =========================
        // LEFT TRIGGER / GRIP: UP / DOWN
        // =========================
        if (left.TryGetFeatureValue(CommonUsages.trigger, out float leftTrigger))
        {
            renderingOffset += Vector3.up * leftTrigger * moveSpeed;
        }

        if (left.TryGetFeatureValue(CommonUsages.grip, out float leftGrip))
        {
            renderingOffset += Vector3.down * leftGrip * moveSpeed;
        }

        // =========================
        // LEFT BUTTONS: CUBE SIZE
        // =========================
        if (left.TryGetFeatureValue(CommonUsages.primaryButton, out bool xPressed) && xPressed)
        {
            particleSize -= Time.deltaTime * 0.01f;
        }

        if (left.TryGetFeatureValue(CommonUsages.secondaryButton, out bool yPressed) && yPressed)
        {
            particleSize += Time.deltaTime * 0.01f;
        }
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
                    subSocket.ReceiveFrameBytes();
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
                    Debug.LogError(ex.Message);
                }
            }
        }
    }

    void ThreadSafeRead()
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
        ThreadSafeRead();


        if (particles == null || particles.Length != latestPointCount)
            particles = new ParticleSystem.Particle[latestPointCount];

        float scale = 0.001f;

        for (int i = 0; i < latestPointCount; i++)
        {
            int idx = i * 3;

            Vector3 localPos = new Vector3(
                latestxyzData[idx] * scale,
                latestxyzData[idx + 1] * scale,
                latestxyzData[idx + 2] * scale
            );

            // 🔥 Transformation anwenden
            Vector3 transformed =
                renderingRotation * (localPos * renderingScale) + renderingOffset;

            particles[i].position = transformed;

            particles[i].startColor = new Color32(
                latestrgbData[idx],
                latestrgbData[idx + 1],
                latestrgbData[idx + 2],
                255
            );

            particles[i].startSize = particleSize;
            particles[i].remainingLifetime = float.MaxValue;
        }

        ps.SetParticles(particles, latestPointCount);


        timeAccumulator += Time.unscaledDeltaTime;
        frameCount++;

        if (timeAccumulator >= 1.0f)
        {
            float fps = frameCount / timeAccumulator;

            text.text = $"{fps:0} FPS";
            UnityEngine.Debug.Log($"{fps:0} FPS");

            // Reset für nächste Sekunde
            timeAccumulator = 0f;
            frameCount = 0;
        }
    }

    // ============================
    // DISCOVERY
    // ============================
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