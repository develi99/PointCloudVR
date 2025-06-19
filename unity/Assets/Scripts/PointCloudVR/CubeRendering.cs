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
using UnityEngine.XR.OpenXR.Features.Meta;
using UnityEngine.Android;


public class InstancedCubeRenderer : MonoBehaviour
{
    //Depth and RGB data
    private Texture2D rgbTexture;
    private ComputeBuffer depthBuffer;
    private uint[] depth;


    //These Parameters are used in Start for init. The real size is set dynamically during runtime
    //ts means thread safe
    private int width_ts = 1, width = 1;
    private int height_ts = 1, height = 1;


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
    private Vector4[] colors;

    private ComputeBuffer matrixBuffer;
    private ComputeBuffer argsBuffer;
    private ComputeBuffer colorBuffer;
    private Bounds renderBounds;


    //Additional Variables, ts means thread safe
    public ComputeShader pointCloudCompute;
    public float CubeSize = 0.01f;
    public float scale = 100.0f;
    private float cullmin_ts = 0.01f, cullmin = 0.01f;
    private float cullmax_ts = 10.0f, cullmax = 10.0f;
    private float x_cull_ts = 2.0f, x_cull = 2.0f;
    private float y_cull_ts = 2.0f, y_cull = 2.0f;


    //Controller handling
    public float moveSpeedPerSecond = 0.2f;
    public float rotationSpeedDegreesPerSecond = 25.0f;
    public float scalingRatePerSecond = 1.5f;
    public float cubeSizePerSecond = 0.01f;

    private Vector3 renderingLocation = new Vector3(0,0,0);
    private Quaternion renderingRotation = Quaternion.identity;
    private Matrix4x4 poseMatrix = Matrix4x4.identity;


    //Camera intrinsics, will be changed during runtime, ts means thread safe
    private float fx_ts = 591.4252f, fx = 591.4252f;
    private float fy_ts = 591.4252f, fy = 591.4252f;
    private float cx_ts = 320.1326f, cx = 320.1326f;
    private float cy_ts = 239.1477f, cy = 239.1477f;


    /*
     * Initalize 
     */
    void Start()
    {
        if (!Permission.HasUserAuthorizedPermission(Permission.Camera))
        {
            Permission.RequestUserPermission(Permission.Camera);
        }

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


    /*
     * hera all buffers are create depending on width and height
     */
    void InitBuffers() {
        UnityEngine.Debug.Log("Init with size " + width + " " + height);

        rgbTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
        depthBuffer = new ComputeBuffer(width * height, sizeof(uint));
        depth = new uint[width * height];

        matrixBuffer = new ComputeBuffer(width * height, Marshal.SizeOf(typeof(Matrix4x4)));
        instancedMaterial.SetBuffer("matrixBuffer", matrixBuffer);

        instancedMaterial.SetTexture("_ColorTex", rgbTexture);
        instancedMaterial.SetInt("_Width", width);

        argsBuffer = new ComputeBuffer(1, 5 * sizeof(uint), ComputeBufferType.IndirectArguments);
        uint[] args = new uint[5] {
            (cubeMesh != null) ? cubeMesh.GetIndexCount(0) : 0,
            (uint) (width * height),
            (cubeMesh != null) ? cubeMesh.GetIndexStart(0) : 0,
            (cubeMesh != null) ? cubeMesh.GetBaseVertex(0) : 0,
            0
        };
        argsBuffer.SetData(args);
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

        // --- RIGHT TRIGGER for scaling up ---
        /*if (rightController.TryGetFeatureValue(CommonUsages.trigger, out float rightTriggerValue))
        {
            if (rightTriggerValue > 0.1f) // threshold
                scale *= scaleFactorUp;
        }

        // --- RIGHT GRIP for scaling down ---
        if (rightController.TryGetFeatureValue(CommonUsages.grip, out float rightGripValue))
        {
            if (rightGripValue > 0.1f) // threshold
                scale *= scaleFactorDown;
        }*/



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


    /*
     * 1. Handle Inputs and calculate Pose Matrix
     * 2. Thread-safe reading of the Data, that comes from the networking thread
     * 3. Update buffers
     * 4. Draw the Cubes
     * 5. Logging
     */
    void Update()
    {
        // 1. Controller Handling
        controllerHandling();

        // 2. Thread-safe reading
        lock (Lock)
        {
            latestDepthBytes = sharedDepthBytes;
            latestRgbBytes = sharedRgbBytes;
            sharedRgbBytes = null;
            sharedDepthBytes = null;

            width = width_ts;
            height = height_ts;

            fx = fx_ts;
            fy = fy_ts;
            cx = cx_ts;
            cy = cy_ts;

            cullmax = cullmax_ts;
            cullmin = cullmin_ts;

            x_cull = x_cull_ts;
            y_cull = y_cull_ts;
        }

        // 3. if new data available, update all buffers
        if (latestDepthBytes != null && latestRgbBytes != null)
        {
            if (rgbTexture.width != width || rgbTexture.height != height)
                InitBuffers();

            rgbTexture.LoadImage(latestRgbBytes);
            setDepthBuffer();
            DispatchComputeShader();

            latestDepthBytes = null;
            latestRgbBytes = null;
        }

        // 4. Draw all cubes
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


    /*
     * Reads the depth buffer and sets it in the coorect format to the ComputeBuffer on the GPU
     */
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

        depthBuffer.SetData(depth);

    }


    /*
     * Starts the Computeshader to calculate the Pointcloud
     */
    void DispatchComputeShader()
    {
        int kernel = pointCloudCompute.FindKernel("CSMain");

        pointCloudCompute.SetBuffer(kernel, "depthBuffer", depthBuffer);
        pointCloudCompute.SetBuffer(kernel, "matrixBuffer", matrixBuffer);

        pointCloudCompute.SetInt("_Width", width);
        pointCloudCompute.SetInt("_Height", height);
        pointCloudCompute.SetFloat("_Fx", fx);
        pointCloudCompute.SetFloat("_Fy", fy);
        pointCloudCompute.SetFloat("_Cx", cx);
        pointCloudCompute.SetFloat("_Cy", cy);
        pointCloudCompute.SetFloat("_Scale", scale);
        pointCloudCompute.SetFloat("_CubeSize", CubeSize);
        pointCloudCompute.SetFloat("_CullMinZ", cullmin);
        pointCloudCompute.SetFloat("_CullMaxZ", cullmax);
        pointCloudCompute.SetFloat("_CullX", x_cull);
        pointCloudCompute.SetFloat("_CullY", y_cull);

        poseMatrix = Matrix4x4.TRS(renderingLocation, renderingRotation, Vector3.one);
        pointCloudCompute.SetMatrix("_PoseMatrix", poseMatrix);

        int threadGroupsX = Mathf.CeilToInt(width / 8.0f);
        int threadGroupsY = Mathf.CeilToInt(height / 8.0f);
        pointCloudCompute.Dispatch(kernel, threadGroupsX, threadGroupsY, 1);
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


    /*
     * handles all incoming messages from the zmq server
     */
    void ZmqListener()
    {
        AsyncIO.ForceDotNet.Force();

        string serverIp = null;
        bool serverfound = false;
        while(!serverfound && isRunning) {

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
            UnityEngine.Debug.Log("Runs");
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

                    UnityEngine.Debug.Log("Empfangen");

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
                    offset += depthLen;

                    // 6. camera intrinsics
                    if (lastMsg.Length < offset + 16) return;
                    float l_fx = BitConverter.ToSingle(lastMsg, offset); offset += 4;
                    float l_fy = BitConverter.ToSingle(lastMsg, offset); offset += 4;
                    float l_cx = BitConverter.ToSingle(lastMsg, offset); offset += 4;
                    float l_cy = BitConverter.ToSingle(lastMsg, offset); offset += 4;

                    // 7. Culling parameters
                    if (lastMsg.Length < offset + 16) return;
                    float l_cullmin = BitConverter.ToSingle(lastMsg, offset); offset += 4;
                    float l_cullmax = BitConverter.ToSingle(lastMsg, offset); offset += 4;
                    float l_x_cull = BitConverter.ToSingle(lastMsg, offset); offset += 4;
                    float l_y_cull = BitConverter.ToSingle(lastMsg, offset); offset += 4;

                    // Thread-Safe assignments
                    lock (Lock)
                    {
                        sharedRgbBytes = rgbBytes;
                        sharedDepthBytes = depthBytes;

                        this.width_ts = l_width;
                        this.height_ts = l_height;

                        this.fx_ts = l_fx;
                        this.fy_ts = l_fy;
                        this.cx_ts = l_cx;
                        this.cy_ts = l_cy;

                        this.cullmax_ts = l_cullmax;
                        this.cullmin_ts = l_cullmin;
                        this.x_cull_ts = l_x_cull;
                        this.y_cull_ts = l_y_cull;
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


    /*
     * After Focus has changed, it's important to reset the buffers otherwise it won't work
     */
    void OnApplicationFocus(bool hasFocus)
    {
        if (hasFocus)
        {
            InitBuffers(); 
        }
    }


    /*
    * After OnApplicationPause, it's important to reset the buffers otherwise it won't work
    */
    void OnApplicationPause(bool pause)
    {
        if(!pause)
        {
            InitBuffers();
        }
    }
}
