using NetMQ;
using NetMQ.Sockets;
using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.XR;

public class ZmqDepthRGBClientImprovedRawFlipped : MonoBehaviour
{
    public Material pointCloudMaterial;

    private Texture2D rgbTexture;
    private ushort[] depthUShortArray;

    private Mesh pointCloudMesh;
    private Vector3[] vertices;
    private Color[] colors;
    private int[] indices;

    //These Parameters are used in Start for init. The real size is set dynamically at 
    //runtime but i should never exceed than the size specified here
    private int width = 1280;
    private int height = 720;

    /* 
     * Here the arrived message from zmq server is saved, be carefull it don't contains 
     * the values, it contains the bytes of jpeg (rgb) and png (depth)
     * Only the actual data is saved, without queue because it isn't needed, only 
     * the actual data ist necessary
     */
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

    private byte[] lastMsg;

    private GameObject pcObj;

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
        pcObj = new GameObject("PointCloud");
        MeshFilter mf = pcObj.AddComponent<MeshFilter>();
        MeshRenderer mr = pcObj.AddComponent<MeshRenderer>();
        mf.mesh = pointCloudMesh;
        mr.material = pointCloudMaterial; // Material is public and set from outside


        //subSocket.Options.ReceiveHighWatermark = 1;
        isRunning = true;
        listenerThread = new Thread(ZmqListener);
        listenerThread.IsBackground = true;
        listenerThread.Start();
    }

    public float moveSpeed = 0.1f;

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

    void Update() {

        var rightController = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        var leftController = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);

        if (!rightController.isValid)
            UnityEngine.Debug.LogWarning("Right controller not valid!");

        if (!leftController.isValid)
            UnityEngine.Debug.LogWarning("Left controller not valid!");

        // Bewegung entlang X-Achse
        if (IsButtonPressed(rightController, CommonUsages.primaryButton)) // A
            pcObj.transform.position += Vector3.right * moveSpeed;

        if (IsButtonPressed(leftController, CommonUsages.primaryButton)) // X
            pcObj.transform.position += Vector3.left * moveSpeed;

        // Bewegung entlang Y-Achse
        if (IsButtonPressed(rightController, CommonUsages.secondaryButton)) // B
            pcObj.transform.position += Vector3.up * moveSpeed;

        if (IsButtonPressed(leftController, CommonUsages.secondaryButton)) // Y
            pcObj.transform.position += Vector3.down * moveSpeed;

        // Bewegung entlang Z-Achse
        if (IsButtonPressed(rightController, CommonUsages.gripButton)) // rechter Griff
            pcObj.transform.position += Vector3.forward * moveSpeed;

        if (IsButtonPressed(leftController, CommonUsages.gripButton)) // linker Griff
            pcObj.transform.position += Vector3.back * moveSpeed;


        // Read latest rgb and depth bytes
        lock (Lock) {
            latestRgbBytes = sharedRgbBytes;
            sharedRgbBytes = null;

            latestDepthBytes = sharedDepthBytes;
            sharedDepthBytes = null;
        }


        // If both set, update Point Cloud and wait for new pictures
        if (latestRgbBytes != null && latestDepthBytes != null)
        {
            rgbTexture.LoadImage(latestRgbBytes);
            Buffer.BlockCopy(latestDepthBytes, 0, depthUShortArray, 0, latestDepthBytes.Length);

            UpdatePointCloud();

            latestRgbBytes = null;
            latestDepthBytes = null;

            //Calculate FPS
            /*
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
            */
        }
    }

    void UpdatePointCloud()
    {

        float toMeter = 0.001f; // The Values are between 0-1, normally they are in mm (0-65635) with that code they are converted to meter
        float scale = 50f;
        float maxDepth = 10.0f;    // Optional: maximale Tiefe in Metern

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

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int i = y * width + x;

                // Flip nur bei depth
                int flippedY = height - 1 - y;
                int flippedI = flippedY * width + x;
                float z = depthUShortArray[flippedI] * toMeter;


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

                vertices[i] = new Vector3(X * scale, Y * scale, z * scale);
                colors[i] = rgbPixels[i]; // ebenfalls: nicht mehr flippedI verwenden

                if (z <= 0 || z > maxDepth)
                {
                    colors[i] = Color.red;
                }
            }
        }

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
    void ZmqListener()
    {
        AsyncIO.ForceDotNet.Force();

        using (subSocket = new PullSocket(">tcp://192.168.0.208:5555"))
        {
            //subSocket.Options.ReceiveHighWatermark = 1;
            while (isRunning)
            {
                try
                {
                    while (subSocket.TryReceiveFrameBytes(TimeSpan.Zero, out byte[] msg)) {
                        lastMsg = msg;
                    }
                    if (lastMsg == null) continue;
                    if (lastMsg.Length < 8)
                    {
                        UnityEngine.Debug.LogWarning("[ZMQ] Nachricht zu kurz für Header");
                        continue;
                    }
                    
                    int rgbLen = BitConverter.ToInt32(lastMsg, 0);
                    if (lastMsg.Length < 4 + rgbLen + 4) continue;

                    byte[] rgbBytes = new byte[rgbLen];
                    Buffer.BlockCopy(lastMsg, 4, rgbBytes, 0, rgbLen);

                    int depthLen = BitConverter.ToInt32(lastMsg, 4 + rgbLen);
                    if (lastMsg.Length < 4 + rgbLen + 4 + depthLen) continue;

                    byte[] depthBytes = new byte[depthLen];
                    Buffer.BlockCopy(lastMsg, 4 + rgbLen + 4, depthBytes, 0, depthLen);

                    lock (Lock)
                    {
                        sharedRgbBytes = rgbBytes;
                        sharedDepthBytes = depthBytes;
                    }

                    lastMsg = null;
                }
                
                catch (Exception ex)
                {
                    UnityEngine.Debug.LogError("[ZMQ] Fehler im Listener: " + ex.Message);
                    break;
                }
            }
        }
    }*/

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
                UnityEngine.Debug.Log("[ZMQ] Server gefunden: " + discoveredIp);
            }
        }
        catch (SocketException)
        {
            UnityEngine.Debug.LogWarning("[ZMQ] Kein Server gefunden.");
        }

        client.Close();
        return discoveredIp;
    }

    void ZmqListener()
    {
        AsyncIO.ForceDotNet.Force();
        string serverIp = FindServer();
        if (string.IsNullOrEmpty(serverIp))
        {
            UnityEngine.Debug.LogError("[ZMQ] Kein Server gefunden.");
            return;
        }

        using (subSocket = new PullSocket($">tcp://{serverIp}:5555"))
        {
            //subSocket.Options.ReceiveHighWatermark = 1;
            while (isRunning)
            {
                try
                {
                    while (subSocket.TryReceiveFrameBytes(TimeSpan.Zero, out byte[] msg))
                    {
                        lastMsg = msg;
                    }
                    if (lastMsg == null) continue;
                    if (lastMsg.Length < 8)
                    {
                        UnityEngine.Debug.LogWarning("[ZMQ] Nachricht zu kurz für Header");
                        continue;
                    }
                    int offset = 0;

                    // 1. Größe (Width & Height)
                    if (lastMsg.Length < offset + 8) return;
                    int width = BitConverter.ToInt32(lastMsg, offset); offset += 4;
                    int height = BitConverter.ToInt32(lastMsg, offset); offset += 4;

                    // 2. RGB-Länge
                    if (lastMsg.Length < offset + 4) return;
                    int rgbLen = BitConverter.ToInt32(lastMsg, offset); offset += 4;

                    // 3. RGB-Daten
                    if (lastMsg.Length < offset + rgbLen) return;
                    byte[] rgbBytes = new byte[rgbLen];
                    Buffer.BlockCopy(lastMsg, offset, rgbBytes, 0, rgbLen);
                    offset += rgbLen;

                    // 4. Depth-Länge
                    if (lastMsg.Length < offset + 4) return;
                    int depthLen = BitConverter.ToInt32(lastMsg, offset); offset += 4;

                    // 5. Depth-Daten
                    if (lastMsg.Length < offset + depthLen) return;
                    byte[] depthBytes = new byte[depthLen];
                    Buffer.BlockCopy(lastMsg, offset, depthBytes, 0, depthLen);

                    // ✅ Speichern
                    lock (Lock)
                    {
                        sharedRgbBytes = rgbBytes;
                        sharedDepthBytes = depthBytes;
                        this.width = width;
                        this.height = height;
                    }
                    lastMsg = null;
                } catch (Exception ex)
                {
                    UnityEngine.Debug.LogError("[ZMQ] Fehler im Listener: " + ex.Message);
                    break;
                }
            }
        }
    }
}