using NetMQ;
using NetMQ.Sockets;
using System;
using System.CodeDom;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.OpenXR;
using UnityEngine.XR.OpenXR.Features;
using UnityEngine.Android;


public class PointCloud : MonoBehaviour
{
    private readonly object Lock = new object();

    private Thread listenerThread;
    private bool isRunning = false;
    private SubscriberSocket subSocket;


    //FPS Log
    public bool logPerformance = true;
    private int frames = 0;
    private float lastTimeFrames = 0.0f;


    //Rendering Stuff
    public Material instancedMaterial;

    private Mesh cubeMesh;
    private Matrix4x4[] matrices;
    private Vector4[] colors;

    private ComputeBuffer matrixBuffer;
    private ComputeBuffer argsBuffer;
    private ComputeBuffer colorBuffer;
    private Bounds renderBounds;


    //Additional Variables, ts means thread safe
    public float CubeSize = 0.01f;
    public float scale = 1.0f;


    //Controller handling
    public float moveSpeedPerSecond = 0.2f;
    public float rotationSpeedDegreesPerSecond = 25.0f;
    public float scalingRatePerSecond = 1.5f;
    public float cubeSizePerSecond = 0.01f;

    private Vector3 renderingLocation = new Vector3(0, 0, 0);
    private Quaternion renderingRotation = Quaternion.identity;
    private Matrix4x4 poseMatrix = Matrix4x4.identity;
    private bool controlsEnabled;

    private GameObject obj;

    //PointCloudStuff
    private short[] sharedxyzData;
    private short[] latestxyzData;
    private byte[] sharedrgbData;
    private byte[] latestrgbData;
    private int sharedPointCount = 1;
    private int latestPointCount = 1;
    private int acutualPointCount = 1;
    private int numberReceivedPoints = 0;
    private int numberReceivedPoints_ts = 0;

    private void Start()
    {
        //Read GameObject pointcloudtest
        obj = GameObject.Find("PointCloudTest");


        //Create necessary Objects for rendering
        GameObject temp = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cubeMesh = temp.GetComponent<MeshFilter>().sharedMesh;
        Destroy(temp);
        renderBounds = new Bounds(Vector3.zero, Vector3.one * 1000);

        //Init changable Buffers, depending from width and height
        InitBuffers();

        //Start network thread
        isRunning = true;
        listenerThread = new Thread(ZmqListener);
        listenerThread.IsBackground = true;
        listenerThread.Start();
    }


    public static float[] ByteArrayToFloatArray(byte[] byteArray)
    {
        if (byteArray == null)
            throw new ArgumentException("Invalid byte array length. Must be a multiple of 4.");

        // Cast the byte array to a ReadOnlySpan<float>
        ReadOnlySpan<byte> byteSpan = byteArray;
        ReadOnlySpan<float> floatSpan = MemoryMarshal.Cast<byte, float>(byteSpan);

        // Convert the ReadOnlySpan<float> to a float[]
        return floatSpan.ToArray();
    }


    void InitBuffers()
    {
        UnityEngine.Debug.Log("PointCloud Init with size " + latestPointCount);

        colorBuffer = new ComputeBuffer(latestPointCount, sizeof(float) * 3);
        instancedMaterial.SetBuffer("colorBuffer", colorBuffer);

        matrixBuffer = new ComputeBuffer(latestPointCount, Marshal.SizeOf(typeof(Matrix4x4)));
        instancedMaterial.SetBuffer("matrixBuffer", matrixBuffer);
        acutualPointCount = latestPointCount;

        argsBuffer = new ComputeBuffer(1, 5 * sizeof(uint), ComputeBufferType.IndirectArguments);
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

            numberReceivedPoints = numberReceivedPoints_ts;
        }
    }

    /*
     * 
     */
    bool IsButtonPressed(InputDevice device, InputFeatureUsage<bool> button)
    {
        bool value = false;

        if (device.TryGetFeatureValue(button, out value))
        {
            UnityEngine.Debug.Log($"Button {button.name} pressed: {value}");
            return value;
        }

        UnityEngine.Debug.LogWarning($"Button {button.name} not found on device {device.name}");
        return false;

    }

    /*
     * Handles the inputs of the Controller 
     */
    void controllerHandling()
    {
        var rightController = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        var leftController = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);

        //It shouldn't depend on fps
        float moveSpeed = moveSpeedPerSecond * Time.deltaTime;
        float rotationSpeedDegrees = rotationSpeedDegreesPerSecond * Time.deltaTime;
        float scaleFactorUp = Mathf.Pow(scalingRatePerSecond, Time.deltaTime);
        float scaleFactorDown = Mathf.Pow(1f / scalingRatePerSecond, Time.deltaTime);
        float cubeSizeSpeed = cubeSizePerSecond * Time.deltaTime;

        if (!rightController.isValid)
            UnityEngine.Debug.LogWarning("Right controller not valid!");

        if (!leftController.isValid)
            UnityEngine.Debug.LogWarning("Left controller not valid!");

        // --- RIGHT STICK for rotation (Yaw and Pitch) ---
        if (rightController.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 rightStick))
        {
            float yaw = rightStick.x * rotationSpeedDegrees;
            float pitch = -rightStick.y * rotationSpeedDegrees;
            Quaternion deltaRotation = Quaternion.Euler(pitch, yaw, 0f);
            renderingRotation = renderingRotation * deltaRotation;
        }

        // --- RIGHT BUTTONS for roll ---
        if (IsButtonPressed(rightController, CommonUsages.primaryButton)) // A: roll right
        {
            Quaternion roll = Quaternion.Euler(0f, 0f, -rotationSpeedDegrees);
            renderingRotation = renderingRotation * roll;
        }

        if (IsButtonPressed(rightController, CommonUsages.secondaryButton)) // B: roll left
        {
            Quaternion roll = Quaternion.Euler(0f, 0f, rotationSpeedDegrees);
            renderingRotation = renderingRotation * roll;
        }

        // --- LEFT STICK for translation X and Z ---
        if (leftController.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 leftStick))
        {
            renderingLocation += new Vector3(leftStick.x, 0f, leftStick.y) * moveSpeed;
        }

        // --- LEFT TRIGGER for moving up ---
        if (leftController.TryGetFeatureValue(CommonUsages.trigger, out float leftTriggerValue))
        {
            if (leftTriggerValue > 0.1f) // threshold
                renderingLocation += Vector3.up * moveSpeed * leftTriggerValue;
        }

        // --- LEFT GRIP for moving down ---
        if (leftController.TryGetFeatureValue(CommonUsages.grip, out float leftGripValue))
        {
            if (leftGripValue > 0.1f) // threshold
                renderingLocation += Vector3.down * moveSpeed * leftGripValue;
        }

        // --- LEFT BUTTONS for cubeSize ---
        if (IsButtonPressed(leftController, CommonUsages.primaryButton)) // X: decrease cubeSize
            CubeSize -= cubeSizeSpeed;

        if (IsButtonPressed(leftController, CommonUsages.secondaryButton)) // Y: increase cubeSize
            CubeSize += cubeSizeSpeed;

    }


    private float logInterval = 1f;
    private float timeSinceLastLog = 0f;
    void Update()
    {
        // 1. Controller Handling
        controllerHandling();

        ThreadSafeReadingPointCloudData();

        if (latestxyzData != null && latestrgbData != null)//if (latestxyzData != null && latestrgbData != null)
        {
            if (acutualPointCount != latestPointCount)
            {
                InitBuffers();
            }
            setColorBuffer();
            setMatrixBufferPointCloud();
            latestxyzData = null;
            latestrgbData = null;
        }


        uint[] args = new uint[5] {
            (cubeMesh != null) ? cubeMesh.GetIndexCount(0) : 0,
            (uint) (numberReceivedPoints),
            (cubeMesh != null) ? cubeMesh.GetIndexStart(0) : 0,
            (cubeMesh != null) ? cubeMesh.GetBaseVertex(0) : 0,
            0
        };
        argsBuffer.SetData(args);


        Graphics.DrawMeshInstancedIndirect(
            cubeMesh,
            0,
            instancedMaterial,
            renderBounds,
            argsBuffer
        );


        // 5. Logging
        if (logPerformance)
            logFPS();

        timeSinceLastLog += Time.deltaTime;
        if (timeSinceLastLog >= logInterval)
        {
            UnityEngine.Debug.Log("position manager: " + obj.transform.position);
            UnityEngine.Debug.Log("position rotation: " + obj.transform.rotation);
            timeSinceLastLog = 0f;
        }
    }


    /*
     * Logging FPS, just debugging
     */
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


    void setColorBuffer()
    {
        // In Vector3-Array konvertieren
        Vector3[] rgbVectors = new Vector3[numberReceivedPoints];
        for (int i = 0; i < numberReceivedPoints; i++)
        {
            rgbVectors[i] = new Vector3(
                latestrgbData[i * 3 + 0] / 255f,
                latestrgbData[i * 3 + 1] / 255f,
                latestrgbData[i * 3 + 2] / 255f
            );
        }
        colorBuffer.SetData(rgbVectors);
    }

    void setMatrixBufferPointCloud()
    {
        Matrix4x4[] matrices = new Matrix4x4[numberReceivedPoints];
        UnityEngine.Debug.Log(numberReceivedPoints);
        poseMatrix = Matrix4x4.TRS(obj.transform.position, obj.transform.rotation, Vector3.one);
        for (int i = 0; i < numberReceivedPoints; i++)
        {
            // Indizes im Float-Array (3 Werte pro Punkt)
            int baseIndex = i * 3;

            float x = latestxyzData[baseIndex] / 1000f;
            float y = latestxyzData[baseIndex + 1] / 1000f;
            float z = latestxyzData[baseIndex + 2] / 1000f;

            Matrix4x4 matrix = poseMatrix * Matrix4x4.TRS(
                new Vector3(x * scale, y * scale, z * scale),   // Position
                Quaternion.identity,                            // Keine Rotation
                Vector3.one * CubeSize * scale                  // Einheitliche Skalierung, z. B. 0.01f
            );

            matrices[i] = matrix;
        }

        // ComputeBuffer setzen (zuvor korrekt initialisiert!)
        matrixBuffer.SetData(matrices);
    }


    /*
     * Finds the coorect server in the Subnet, so the server ip address is not hardcoded
     */
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
        while (!serverfound && isRunning)
        {
            serverIp = FindServer();
            if (string.IsNullOrEmpty(serverIp))
            {
                UnityEngine.Debug.LogError("[ZMQ] No Server found.");
                System.Threading.Thread.Sleep(1000);
                continue;
            }
            serverfound = true;
        }

        using (subSocket = new SubscriberSocket())
        {
            subSocket.Connect($"tcp://{serverIp}:5555");
            subSocket.Subscribe("PointCloud"); // <<< Das Topic abonnieren

            UnityEngine.Debug.Log("[ZMQ] Subscriber verbunden mit Topic 'PointCloud'");

            while (isRunning)
            {
                try
                {
                    // Multipart-Nachricht empfangen: [Topic, Daten]
                    string topic = subSocket.ReceiveFrameString(); // Frame 1: Topic
                    byte[] lengthBytes = subSocket.ReceiveFrameBytes();   // Frame 2: numPoints (4 Bytes)
                    byte[] xyzBytes = subSocket.ReceiveFrameBytes();    // Frame 3: xyz
                    byte[] rgbBytes = subSocket.ReceiveFrameBytes();    // Frame 4: rgb

                    // Länge aus Frame 2 auslesen (int32, little endian)
                    int numPoints = BitConverter.ToInt32(lengthBytes, 0);

                    int xyzCount = xyzBytes.Length / sizeof(short);
                    int rgbCount = rgbBytes.Length / sizeof(byte);
                    if (xyzCount % 3 != 0 || xyzCount != rgbCount)
                    {
                        UnityEngine.Debug.LogWarning("[ZMQ] Ungültige Punktwolkendaten empfangen (Größe passt nicht)");
                        return;
                    }


                    // In float[] umwandeln
                    short[] xyzData = new short[xyzCount];
                    byte[] rgbData = new byte[rgbCount];

                    Buffer.BlockCopy(xyzBytes, 0, xyzData, 0, xyzBytes.Length);
                    Buffer.BlockCopy(rgbBytes, 0, rgbData, 0, rgbBytes.Length);

                    // Thread-safe speichern
                    lock (Lock)
                    {
                        this.sharedxyzData = xyzData;
                        this.sharedrgbData = rgbData;
                        this.sharedPointCount = numPoints;
                        this.numberReceivedPoints_ts = xyzCount / 3;
                    }

                }
                catch (Exception ex)
                {
                    UnityEngine.Debug.LogError("[ZMQ] Fehler im Subscriber: " + ex.Message);
                    break;
                }
            }
        }
    }


    /*
     * Cleans all buffers, stops the network-thread and closes the connection
     */
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

}