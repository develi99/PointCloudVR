using UnityEngine;
using NetMQ;
using NetMQ.Sockets;
using System;
using System.Threading;

public class ZmqFrameReceiver : MonoBehaviour
{
    public string host = "192.168.0.208";  // Server-IP
    public int port = 5555;

    public int width = 640;
    public int height = 480;

    private Texture2D texture;
    private Renderer rend;
    private Thread receiveThread;
    private bool running = true;
    private byte[] latestFrame;
    private object frameLock = new object();

    void Start()
    {
        rend = GetComponent<Renderer>();
        texture = new Texture2D(width, height, TextureFormat.RGB24, false);
        rend.material.mainTexture = texture;

        receiveThread = new Thread(ReceiveLoop);
        receiveThread.Start();
    }

    void ReceiveLoop()
    {
        AsyncIO.ForceDotNet.Force();  // Wichtig für Unity

        using (var subSocket = new PullSocket())
        {
            string address = $"tcp://{host}:{port}";
            subSocket.Connect(address);
            Debug.Log($"[ZMQ] Verbunden mit {address}");

            while (running)
            {
                try
                {
                    byte[] data = subSocket.ReceiveFrameBytes();

                    if (data.Length < 4) continue;

                    int frameLen = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
                    if (frameLen + 4 > data.Length) continue;

                    byte[] jpg = new byte[frameLen];
                    Array.Copy(data, 4, jpg, 0, frameLen);

                    lock (frameLock)
                    {
                        latestFrame = jpg;
                    }
                }
                catch (Exception e)
                {
                    Debug.LogWarning($"[ZMQ] Fehler beim Empfang: {e.Message}");
                }
            }
        }
    }

    void Update()
    {
        lock (frameLock)
        {
            if (latestFrame != null)
            {
                texture.LoadImage(latestFrame);
                texture.Apply();
                latestFrame = null;
            }
        }
    }

    void OnDestroy()
    {
        running = false;
        receiveThread?.Join();
        NetMQConfig.Cleanup();
    }
}
